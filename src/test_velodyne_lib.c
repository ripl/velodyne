#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>

#include <gsl/gsl_blas.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <velodyne/velodyne.h>
#include <velodyne/velodyne_extractor.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <GL/gl.h>

//#include <lcmtypes/ripl_xyz_point_list_t.h>

typedef struct _state_t state_t;

struct _state_t {
    lcm_t *lcm;
    GMainLoop *mainloop;
    BotParam   *b_server;
    velodyne_extractor_state_t *velodyne; 
    BotFrames *frames;
    bot_lcmgl_t *lcmgl;
};

static void velodyne_update_handler(int64_t utime, void *user)
{
    state_t *s = (state_t *)user;
    fprintf(stderr," ++++++++ Callback Called\n");
    xyzr_point_list_t *points = velodyne_extract_points_and_distance_frame(s->velodyne, "body");
    
    destroy_xyzr_list(points);
}

//doesnt do anything right now - timeout function
gboolean heartbeat_cb_1 (gpointer data)
{
    state_t *s = (state_t *)data;
    
    velodyne_full_scan_t *ret = velodyne_extract_new_scans(s->velodyne);//, &p_list); 

    if(ret != NULL){
        /*for(int i=0; i < ret->no_points ; i+=1000){
            fprintf(stderr, "\t %f,%f,%f\n", ret->points[i].xyz[0], 
                    ret->points[i].xyz[1],
                    ret->points[i].xyz[2]);
                    }*/
        velodyne_full_scan_t_destroy(ret);
    } 
    //return true to keep running
    return TRUE;
}

//doesnt do anything right now - timeout function
gboolean heartbeat_cb (gpointer data)
{
    state_t *s = (state_t *)data;
    
    xyz_point_list_t *ret = velodyne_extract_new_points(s->velodyne);//, &p_list); 

    if(ret != NULL){
        fprintf(stderr, "Size of Points : %d \n", ret->no_points);

        double sensor_to_local[12];
        if (!bot_frames_get_trans_mat_3x4_with_utime (s->frames, "VELODYNE",
                                                      "local", ret->utime, 
                                                      sensor_to_local)) {
            fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
            return TRUE;        
        }

        //draw some points 
        bot_lcmgl_t *lcmgl = s->lcmgl;
        double xy[8] = {-4,0, 4,0, 0,4, 0, -4};

        bot_lcmgl_color3f(s->lcmgl, 1, 1, 0);
        bot_lcmgl_point_size(s->lcmgl, 8);
        bot_lcmgl_line_width(s->lcmgl, 8);
        bot_lcmgl_begin(s->lcmgl, GL_LINES);
        //fprintf(stderr,"Processing\n");
        for(int i=0; i < 4; i++){
            double pos[3] = {xy[2*i], xy[2*i+1], 0};
            double g_pos[3];
            bot_vector_affine_transform_3x4_3d (sensor_to_local, pos, g_pos);
            bot_lcmgl_vertex3f(s->lcmgl, g_pos[0], g_pos[1], g_pos[2]);
        }
    
        bot_lcmgl_end(s->lcmgl);
        bot_lcmgl_switch_buffer(s->lcmgl);

        /*ripl_xyz_point_list_t msg;
        msg.utime = bot_timestamp_now(); 
        msg.no_points = ret->no_points;
    
        msg.points = (ripl_xyz_point_t *)calloc(msg.no_points, sizeof(ripl_xyz_point_t));

        for (size_t k = 0; k < ret->no_points; ++k){
            msg.points[k].xyz[0] = ret->points[k].xyz[0];//cloud_p->points[k].x; 
            msg.points[k].xyz[1] = ret->points[k].xyz[1]; 
            msg.points[k].xyz[2] = ret->points[k].xyz[2]; 
        }

        //publish
        ripl_xyz_point_list_t_publish(s->lcm, "PCL_XYZ_LIST", &msg);
        free(msg.points);    */

        /*for(int i=0; i < ret->no_points ; i+=1000){
            fprintf(stderr, "\t %f,%f,%f\n", ret->points[i].xyz[0], 
                    ret->points[i].xyz[1],
                    ret->points[i].xyz[2]);
                    }*/
        destroy_xyz_list(ret);
    } 
    //return true to keep running
    return TRUE;
}

int 
main(int argc, char **argv)
{

    g_thread_init(NULL);
    setlinebuf (stdout);
    state_t *state = (state_t*) calloc(1, sizeof(state_t));

    state->lcm =  bot_lcm_get_global(NULL);
    state->velodyne = velodyne_extractor_init(state->lcm, &velodyne_update_handler, state);
    state->b_server = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->b_server);
    state->lcmgl = bot_lcmgl_init(state->lcm,"velodyne_packet_test");

    state->mainloop = g_main_loop_new( NULL, FALSE );  
  
    if (!state->mainloop) {
	printf("Couldn't create main loop\n");
	return -1;
    }

    //add lcm to mainloop 
    bot_glib_mainloop_attach_lcm (state->lcm);

    /* heart beat*/
    //g_timeout_add (100, heartbeat_cb, state);

    //adding proper exiting 
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);
    
    fprintf(stderr, "Starting Main Loop\n");

    ///////////////////////////////////////////////
    g_main_loop_run(state->mainloop);
  
    bot_glib_mainloop_detach_lcm(state->lcm);
}
