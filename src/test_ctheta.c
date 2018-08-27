#include <stdio.h>
#include <inttypes.h>

#include <lcm/lcm.h>
#include <lcmtypes/exlcm_example_t.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <velodyne/velodyne.h>
#include <path_utils/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include "lcmtypes/senlcm_velodyne_t.h"
#include "lcmtypes/senlcm_velodyne_list_t.h"


int main (int argc, char **argv)
{
    int verbose = 0;
    char *lcm_fname = NULL;
    
    lcm_fname = argv[argc -1];

    // now, generate the images
    lcm_eventlog_t *src_log = lcm_eventlog_create (lcm_fname, "r");
    if (!src_log) {
        fprintf (stderr, "Unable to open source logfile %s\n", lcm_fname);
        return 1;
    }

    int have_ctheta = 0;
    double max_delta = 0;
    double last_ctheta = 0;
    int64_t last_utime = 0;
    
    for (lcm_eventlog_event_t *event = lcm_eventlog_read_next_event (src_log);
         event != NULL;
         event = lcm_eventlog_read_next_event (src_log)) {
                
        // does the channel name match the specified regex?
        int decode_status;
        if (strcmp ("VELODYNE_LIST", event->channel) == 0) {

            int counter = 0;
            senlcm_velodyne_list_t vlist;
            memset (&vlist, 0, sizeof (senlcm_velodyne_list_t));
            decode_status = senlcm_velodyne_list_t_decode (event->data, 0, event->datalen, &vlist);

            if (decode_status < 0)
                fprintf (stderr, "Error %d decoding message\n", decode_status);
            else {

                for (int i=0; i <vlist.num_packets; i++) {
                    senlcm_velodyne_t v = vlist.packets[i];
             
                    if (v.packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {
                        assert (v.data_len == VELODYNE_DATA_PACKET_LEN);

                        for (int i_f = 0; i_f < VELODYNE_NUM_FIRING_PER_PACKET; i_f++) {
                            
                            double ctheta = VELODYNE_GET_ROT_POS(v.data, VELODYNE_DATA_FIRING_START(i_f));
                            //fprintf (stdout, "i_f = %d, ctheta = %.4f degrees\n", i_f, ctheta * 180 / M_PI);

                            if (i_f == 0) {
                                int16_t rot_pos_16  = v.data[2] + (v.data[3]<<8);
                                //double test_theta = ((double) rot_pos_16) / 100;
                                //double test_theta = (v.data[2] + (v.data[3]<<8)) / 100;
                                //double test_theta = VELODYNE_GET_ROT_POS(v.data, 0) * 180 / M_PI;
                                int start = 0;
                                double test_theta =  ((v.data[start+2] + (v.data[start+3]<<8)) * VELODYNE_RADIANS_PER_LSB);
                                double ctheta_deg = ctheta * 180 / M_PI;
                                //if (fabs (ctheta_deg - test_theta) > 0.1)
                                    //fprintf (stdout, "i_f = %d, ctheta_deg = %.4f, test_theata = %.4f\n", i_f, ctheta_deg, test_theta);
                            }

                            if (have_ctheta) {
				double delta = bot_mod2pi_ref (0, ctheta - last_ctheta);
                                if (fabs(delta) > 0.01)
                                    fprintf (stdout, "i_f = %d, ctheta (%.4f) - last_ctheta (%.4f) = %.4f\n", 
                                             i_f, ctheta * 180/M_PI, last_ctheta * 180/M_PI, delta * 180 / M_PI);

				int16_t rot_pos_16  = v.data[2] + (v.data[3]<<8);
                                double test_theta = ((double) rot_pos_16) / 100;
                                //fprintf (stdout, "i_f = %d, test_theata = %.4f\n", i_f, test_theta);
                                //fprintf (stdout, "i_f = %d, test_theata = %.4f\n", i_f, (v.data[2] + (v.data[3]<<8)) / 100);
                                if (fabs(ctheta - last_ctheta) > max_delta)
                                    max_delta = ctheta - last_ctheta;
                            }
                            last_ctheta = ctheta;
                            have_ctheta = 1;
                            counter++;
                            
                        }
                        last_utime = v.utime;
                    }

                }
            }
                         
            senlcm_velodyne_list_t_decode_cleanup (&vlist);
        }
        else if (strcmp ("VELODYNE", event->channel) == 0) {

            senlcm_velodyne_t v;
            memset (&v, 0, sizeof (senlcm_velodyne_t));
            decode_status = senlcm_velodyne_t_decode (event->data, 0, event->datalen, &v);

            if (decode_status < 0)
                fprintf (stderr, "Error %d decoding message\n", decode_status);
            else {
                if (v.packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {
                    assert (data_len == VELODYNE_DATA_PACKET_LEN);

                    for (int i_f = 0; i_f < VELODYNE_NUM_FIRING_PER_PACKET; i_f++) {
                        double ctheta = VELODYNE_GET_ROT_POS(v.data, VELODYNE_DATA_FIRING_START(i_f));
                        //fprintf (stdout, "ctheta = %.4f degrees\n", ctheta * 180 / M_PI);
                        //fprintf (stdout, "i_f = %d, ctheta = %.4f degrees\n", i_f, ctheta * 180 / M_PI);
                        
                        if (have_ctheta) {
			    double delta = bot_mod2pi_ref (0, ctheta - last_ctheta);
                                if (fabs(delta) > 0.01)
                                    fprintf (stdout, "i_f = %d, ctheta (%.4f) - last_ctheta (%.4f) = %.4f\n", 
                                             i_f, ctheta * 180/M_PI, last_ctheta * 180/M_PI, delta * 180 / M_PI);
                            if (fabs(ctheta - last_ctheta) > max_delta)
                                max_delta = ctheta - last_ctheta;
                        }
                        last_ctheta = ctheta;
                        have_ctheta = 1;

                    }
                }
            }
                         
            senlcm_velodyne_t_decode_cleanup (&v);
        }
        lcm_eventlog_free_event (event);
    }
    
    lcm_eventlog_destroy (src_log);
    
    fprintf (stdout, "Max ctheta delta = %.4f degrees (%.4f rad)\n", max_delta * 180 / M_PI, max_delta);

    return 0;
}
