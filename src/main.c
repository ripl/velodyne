#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <math.h>
#include <assert.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <velodyne/velodyne.h>

#include "lcmtypes/senlcm_velodyne_t.h"
#include "lcmtypes/senlcm_velodyne_list_t.h"
#include "lcmtypes/bot_core_sensor_status_t.h"

#define UDP_MAX_LEN 1600
#define REPORT_INTERVAL_USECS (1.0 * 1e6)
#define READ_TIMEOUT_USEC (1.0 * 1e6)

#define GAMMA 0.1

#define PUBLISH_HZ 50.0 // Messages will be published

// The velodyne outputs packets at 20kHz
// Store enough packets for each publish with an additional safety margin
#define MAX_PACKET_QUEUE_SIZE ((int) 1.25 * 20000.0 / PUBLISH_HZ)


// define state structure
typedef struct _state_t state_t;
struct _state_t {
    int done;
    int is_daemon;
    lcm_t *lcm;

    GMainLoop *mainloop;

    char *lcm_chan;
    char *velodyne_model;

    GThread *velodyne_read_thread;
    int velodyne_read_thread_exit_flag;
    GMutex * velodyne_read_thread_exit_mutex;

    GAsyncQueue *velodyne_read_thread_queue;

    GThread *lcm_publish_thread;
    int lcm_publish_thread_exit_flag;
    GMutex * lcm_publish_thread_exit_mutex;
    int64_t last_publish_utime;

    GAsyncQueue *packet_queue;

    int verbose;
    bot_timestamp_sync_state_t *tss_data;
    bot_timestamp_sync_state_t *tss_position;

    int64_t last_status_utime;
    int num_packets_since_last_status;
};



uint32_t fread_le32(FILE *f)
{
    uint8_t b[4];
    fread(b, 4, 1, f);
    return b[0] + (b[1]<<8) + (b[2]<<16) + (b[3]<<24);
}

// make and bind a udp socket to an ephemeral port
static int make_udp_socket(int port)
{
    int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
        return -1;

    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(struct sockaddr_in));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(port);
    listen_addr.sin_addr.s_addr = INADDR_ANY;

    int res = bind(sock, (struct sockaddr*) &listen_addr, sizeof(struct sockaddr_in));
    if (res < 0)
        return -2;

    return sock;
}


static void
velodyne_destroy (state_t *self)
{
    if (!self)
        return;

    if (self->tss_data)
        bot_timestamp_sync_free (self->tss_data);
    if (self->tss_position)
        bot_timestamp_sync_free (self->tss_position);

    if (self->lcm_chan)
        free (self->lcm_chan);
    if (self->velodyne_model)
        free (self->velodyne_model);

    if (self)
        free (self);
}


// Velodyne read thread
static void *
velodyne_read_thread (void *user)
{
    state_t *self = (state_t *) user;
    static int64_t msg_count = 0;

    int data_fd = make_udp_socket(VELODYNE_DATA_PORT);
    if (data_fd < 0) {
        printf("Couldn't open data socket UDP socket\n");
        g_thread_exit(NULL);
    }

    int position_fd = 0;
    if (!strcmp (VELODYNE_HDL_32E_MODEL_STR, self->velodyne_model)) {
        position_fd = make_udp_socket(VELODYNE_POSITION_PORT);
        if (position_fd < 0) {
            printf("Couldn't open position socket UDP socket\n");
            g_thread_exit(NULL);
        }
    }

    int data_packet_count = 0;
    int64_t data_last_report_time = bot_timestamp_now();
    double data_hz = 0.0;

    int position_packet_count = 0;
    int64_t position_last_report_time = bot_timestamp_now();
    double position_hz = 0.0;


    static double ctheta_last = 0;


    fd_set set;
    struct timeval tv;

    // packet timestamps give us the microseconds since top of the hour
    // param dev_ticks_per_second = 1e6   The nominal rate at which the device time increments
    // param dev_ticks_wraparound = 3.6e9 Assume that dev_ticks wraps around every wraparound ticks
    // param rate = based on 5sec/day drift rate  An upper bound on the rate error. Should be (1 + eps)
    self->tss_data = bot_timestamp_sync_init (1e6, 3.6e9, 1.00006);
    self->tss_position = bot_timestamp_sync_init (1e6, 3.6e9, 1.00006);

    while (1) {

        void *msg = g_async_queue_try_pop (self->velodyne_read_thread_queue);

        if (msg) {
            // Check the message to see if we should exit
            g_mutex_lock (self->velodyne_read_thread_exit_mutex);
            if (self->velodyne_read_thread_exit_flag) {
                g_mutex_unlock (self->velodyne_read_thread_exit_mutex);
                g_thread_exit (NULL);
            }
            g_mutex_unlock (self->velodyne_read_thread_exit_mutex);
        }


        // Not exiting. Continue to read from Velodyne

        FD_ZERO (&set);
        FD_SET (data_fd, &set);
        if (position_fd){
            FD_SET (position_fd, &set);
        }
        tv.tv_sec = 0;
        tv.tv_usec = READ_TIMEOUT_USEC;

        int ret = select (data_fd+position_fd+1, &set, NULL, NULL, &tv);
        if (ret < 0)
            fprintf (stderr, "ERROR: select()");
        else if (ret == 0) { // Timeout
            fprintf (stdout, "Timeout: no data to read. \n");
        }
        else { // We have data.

            uint8_t buf[UDP_MAX_LEN];
            struct sockaddr_in from_addr;
            socklen_t from_addr_len = sizeof(from_addr);
            ssize_t len;
            uint8_t packet_type;

            if (FD_ISSET(data_fd, &set)) {
                len = recvfrom(data_fd, (void*)buf, UDP_MAX_LEN, 0,
                               (struct sockaddr*) &from_addr, &from_addr_len);
                if (len != VELODYNE_DATA_PACKET_LEN) {
                    fprintf (stderr, "\nERROR: Bad data packet len, expected %d, got %d", VELODYNE_DATA_PACKET_LEN, (int) len);
                    continue;
                }

                packet_type = SENLCM_VELODYNE_T_TYPE_DATA_PACKET;
                data_packet_count++;
            } else if (FD_ISSET(position_fd, &set)){
                len = recvfrom(position_fd, (void*)buf, UDP_MAX_LEN, 0,
                               (struct sockaddr*) &from_addr, &from_addr_len);
                if (len != VELODYNE_POSITION_PACKET_LEN) {
                    fprintf (stderr, "\nERROR: Bad position packet len, expected %d, got %d", VELODYNE_POSITION_PACKET_LEN, (int) len);
                    continue;
                }
                packet_type = SENLCM_VELODYNE_T_TYPE_POSITION_PACKET;
                position_packet_count++;
            } else {
                fprintf (stderr,"ERROR: What?");
            }

            // pull the usec since top of the hour and use for timestamp sync
            // timestamp data is 4 bytes in reverse order
            uint32_t cycle_usec;
            if (packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET){
                cycle_usec = VELODYNE_GET_TIMESTAMP_USEC(buf);
            } else if (packet_type == SENLCM_VELODYNE_T_TYPE_POSITION_PACKET){
                cycle_usec = VELODYNE_GET_TIMESTAMP_USEC(buf);
            }

            senlcm_velodyne_t v;
            if (packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET)
                v.utime = bot_timestamp_sync (self->tss_data, cycle_usec, bot_timestamp_now());
            else if (packet_type == SENLCM_VELODYNE_T_TYPE_POSITION_PACKET)
                v.utime = bot_timestamp_sync (self->tss_position, cycle_usec, bot_timestamp_now());
            v.packet_type = packet_type;
            v.datalen = len;
            v.data = buf;

            // Push the message onto the queue for the LCM publish thread
            g_async_queue_push (self->packet_queue, senlcm_velodyne_t_copy (&v));
            //senlcm_velodyne_t_publish(self->lcm, self->lcm_chan, &v);

        }
    }

    return NULL;
}




static int dropped_packets = 0;
static int64_t dropped_utime;

//pthread - for publishing LCM messages
static void *
lcm_publish_thread (void *user)
{
    state_t *self = (state_t *) user;

    while (1) {

        while (g_async_queue_length (self->packet_queue) > MAX_PACKET_QUEUE_SIZE) {

            void *msg = g_async_queue_pop (self->packet_queue);

            // Check the message to see if we should exit
            g_mutex_lock (self->lcm_publish_thread_exit_mutex);
            if (self->lcm_publish_thread_exit_flag) {
                g_mutex_unlock (self->lcm_publish_thread_exit_mutex);
                g_thread_exit (NULL);
            }
            g_mutex_unlock (self->lcm_publish_thread_exit_mutex);


            // Not exiting. Keep going
            senlcm_velodyne_t *v = (senlcm_velodyne_t *) msg;

            senlcm_velodyne_t_destroy (v);

            int64_t now = bot_timestamp_now();
            dropped_packets++;
            double dt = (now - dropped_utime) / 1000000.0;
            if (dt > 1 || dropped_packets % 1000 == 0) {
                printf("WARNING: dropping velodyne packets (total dropped: %d)\n", dropped_packets);
                dropped_utime = now;
            }
        }

        int64_t now = bot_timestamp_now();

        if ( ((now - self->last_publish_utime) > 1E6/PUBLISH_HZ) && (g_async_queue_length (self->packet_queue) > 0) ) {

            senlcm_velodyne_list_t *vlist = (senlcm_velodyne_list_t *)
                calloc (1, sizeof (senlcm_velodyne_list_t));
            vlist->utime = now;
            vlist->num_packets = 0;
            vlist->packets = NULL;

            int8_t queue_length = g_async_queue_length (self->packet_queue);
            for (int i=0; i < queue_length; i++) {

                void *msg = g_async_queue_pop (self->packet_queue);

                // Check the message to see if we should exit
                if (msg == &(self->lcm_publish_thread_exit_flag)) {
                    g_mutex_lock (self->lcm_publish_thread_exit_mutex);
                    if (self->lcm_publish_thread_exit_flag) {
                        g_mutex_unlock (self->lcm_publish_thread_exit_mutex);
                        //free (vlist.packets);
                        senlcm_velodyne_list_t_destroy (vlist);
                        g_thread_exit (NULL);
                    }
                    g_mutex_unlock (self->lcm_publish_thread_exit_mutex);
                }

                // Not exiting. Keep going
                senlcm_velodyne_t *vmsg = (senlcm_velodyne_t *) msg;

                // Realloate memory for the new message
                vlist->packets = (senlcm_velodyne_t *)
                    realloc (vlist->packets, (vlist->num_packets + 1)
                             * sizeof (senlcm_velodyne_t));

                senlcm_velodyne_t *v = vlist->packets + vlist->num_packets;
                v->utime = vmsg->utime;
                v->packet_type = vmsg->packet_type;
                v->datalen = vmsg->datalen;
                v->data = (uint8_t *) calloc (1, v->datalen * sizeof (uint8_t));
                memcpy (v->data, vmsg->data, vmsg->datalen * sizeof (uint8_t));

                // Increment the number of packets
                vlist->num_packets++;

                // Free up the message
                senlcm_velodyne_t_destroy (vmsg);
            }

            // Update the publish rate
            int64_t dt = now - self->last_publish_utime;

            self->num_packets_since_last_status += vlist->num_packets;

            self->last_publish_utime = now;
            senlcm_velodyne_list_t_publish (self->lcm, "VELODYNE_LIST", vlist);

            // Free the allocated packets
            senlcm_velodyne_list_t_destroy (vlist);
        }

        //int64_t status_now = bot_timestamp_now ();
        if (now - self->last_status_utime > 1e6) {
            bot_core_sensor_status_t status_msg;
            status_msg.utime = now;
            status_msg.sensor_name = "velodyne";
            status_msg.rate = self->num_packets_since_last_status * 1e6 /
                ((double) (now - self->last_status_utime));
            if (!strcmp (VELODYNE_HDL_32E_MODEL_STR, self->velodyne_model))
                status_msg.type = BOT_CORE_SENSOR_STATUS_T_VELODYNE_32;
            else if(!strcmp (VELODYNE_HDL_64E_S1_MODEL_STR, self->velodyne_model))
                status_msg.type = BOT_CORE_SENSOR_STATUS_T_VELODYNE_64;
            else if(!strcmp (VELODYNE_VLP_16_MODEL_STR, self->velodyne_model))
                status_msg.type = BOT_CORE_SENSOR_STATUS_T_VELODYNE_16;


            bot_core_sensor_status_t_publish(self->lcm, "SENSOR_STATUS_VELODYNE", &status_msg);

            self->last_status_utime = now;
            self->num_packets_since_last_status = 0;
        }

    }

}


static void
usage (int argc, char ** argv)
{
    fprintf (stderr, "Usage: %s [OPTIONS]\n"
             "Velodyne driver...\n"
             "\n"
             "Options:\n"
             "-c, --channel   LCM channel name\n"
             "-m, --model     Velodyne model (HDL_32E, HDL_64E, VLP_16)\n"
             "-D, --daemon    Run as daemon (NOT IMPLEMENTED)\n"
             "-h, --help      Print this help and exit\n\n"
             , argv[0]);
}


int main(int argc, char *argv[])
{

    //this needs to be set as the destination IP
    //sudo ifconfig eth0 192.168.3.1 netmask 255.255.0.0 broadcast 192.168.3.255

    // so that redirected stdout won't be insanely buffered.
    setvbuf (stdout, (char *) NULL, _IONBF, 0);


    state_t *self = (state_t *) calloc (1, sizeof (state_t));
    self->is_daemon = 0;

    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
        fprintf (stderr, "Unable to get LCM\n");
        goto failed;
    }


    // Get the model and lcm channel from the param server if available
    BotParam * param = bot_param_new_from_server(self->lcm, 0);
    if (!param) {
        fprintf (stderr, "Unable to get bot_param\n");
        goto failed;
    }
    if (bot_param_get_str (param, "calibration.velodyne.model", &self->velodyne_model) != 0)
        self->velodyne_model = g_strdup (VELODYNE_HDL_32E_MODEL_STR);

    if (bot_param_get_str (param, "calibration.velodyne.channel", &self->lcm_chan) != 0)
        self->lcm_chan = g_strdup("VELODYNE");

    char *optstring = "hDc:m:";
    char c;
    struct option long_opts[] = {
        { "help", no_argument, NULL, 'h' },
        { "daemon", no_argument, NULL, 'D' },
        { "channel", required_argument, NULL, 'c' },
        { "model", required_argument, NULL, 'm' },
        { 0, 0, 0, 0 }
    };

    while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
        case 'c':
            free (self->lcm_chan);
            self->lcm_chan = g_strdup (optarg);
            break;
        case 'm':
            free (self->velodyne_model);
            self->velodyne_model = g_strdup (optarg);
            break;
        case 'v':
            self->verbose = 1;
            break;
        case 'd':
            fprintf (stderr, "Daemon mode not currently supported.\n");
            usage (argc, argv);
            goto failed;
            break;
        case 'h':
        default:
            usage (argc, argv);
            velodyne_destroy (self);
            return 1;
        }
    }


    if ((strcmp(self->velodyne_model, VELODYNE_VLP_16_MODEL_STR)) &&
        (strcmp(self->velodyne_model, VELODYNE_HDL_32E_MODEL_STR)) &&
        (strcmp(self->velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR)) &&
        (strcmp(self->velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))) {

        fprintf (stderr, "Unknown Velodyne model %s specified.\n", self->velodyne_model);
        usage (argc, argv);
        goto failed;
    }

    g_thread_init (NULL);

    // Create the thread that reads data from the Velodyne
    self->velodyne_read_thread = g_thread_create (velodyne_read_thread, self, TRUE, NULL);
    if (!self->velodyne_read_thread) {
        fprintf (stderr, "Error creating glib Velodyne read thread\n");
        goto failed;
    }

    // Create the thread that publishes packet collections
    self->lcm_publish_thread = g_thread_create (lcm_publish_thread, self, TRUE, NULL);
    if (!self->lcm_publish_thread) {
        fprintf (stderr, "Error creating glib LCM publish thread\n");
        goto failed;
    }


    // Create everything for the velodyne read thread
    self->velodyne_read_thread_exit_flag = 0;
    self->velodyne_read_thread_exit_mutex = g_mutex_new();
    self->velodyne_read_thread_queue = g_async_queue_new ();

    // Create elements for LCM publishing thread
    self->lcm_publish_thread_exit_flag = 0;
    self->lcm_publish_thread_exit_mutex = g_mutex_new();

    self->last_publish_utime = 0;

    // Create the queue where the packets will be collected
    self->packet_queue = g_async_queue_new ();

    self->mainloop = g_main_loop_new (NULL, FALSE);
    bot_glib_mainloop_attach_lcm (self->lcm);

    bot_signal_pipe_glib_quit_on_kill (self->mainloop);

    g_main_loop_run (self->mainloop);


    // Exiting
    fprintf (stdout, "Velodyne driver exiting\n");

    // Stop the velodyne read thread
    g_mutex_lock (self->velodyne_read_thread_exit_mutex);
    self->velodyne_read_thread_exit_flag = 1;
    g_mutex_unlock (self->velodyne_read_thread_exit_mutex);
    g_async_queue_push (self->velodyne_read_thread_queue,
                        &(self->velodyne_read_thread_exit_flag));

    g_thread_join (self->velodyne_read_thread);

    g_mutex_free (self->velodyne_read_thread_exit_mutex);


    // Stop the lcm publish thread
    g_mutex_lock (self->lcm_publish_thread_exit_mutex);
    self->lcm_publish_thread_exit_flag = 1;
    g_mutex_unlock (self->lcm_publish_thread_exit_mutex);
    g_async_queue_push (self->packet_queue,
                        &(self->lcm_publish_thread_exit_flag));


    g_mutex_free (self->lcm_publish_thread_exit_mutex);


    bot_glib_mainloop_detach_lcm (self->lcm);

    int num_freed = 0;
    for (void *msg = g_async_queue_try_pop (self->velodyne_read_thread_queue);
         msg; msg = g_async_queue_try_pop (self->velodyne_read_thread_queue)) {

        if (msg == &(self->velodyne_read_thread_exit_flag))
            continue;

        free (msg);
        num_freed++;
    }

    g_async_queue_unref (self->velodyne_read_thread_queue);


    num_freed = 0;
    for (senlcm_velodyne_t *msg = g_async_queue_try_pop (self->packet_queue);
         msg; msg = g_async_queue_try_pop (self->packet_queue)) {

        if (msg == &(self->lcm_publish_thread_exit_flag))
            continue;

        senlcm_velodyne_t_destroy(msg);
        num_freed++;
    }

    g_async_queue_unref (self->packet_queue);

    velodyne_destroy (self);
    return 0;

 failed:
    velodyne_destroy (self);
    return -1;
}
