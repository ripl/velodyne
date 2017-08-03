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

#include <occ_map/VoxelMap.hpp>
#include <lcmtypes/occ_map_voxel_map_t.h>
//#include <lcmtypes/erlcm_xyz_point_list_t.h>

typedef struct _state_t state_t;

struct _state_t {
    lcm_t *lcm;
    GMainLoop *mainloop;
    velodyne_extractor_state_t *velodyne; 
};

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

        double xyz0[3] = { -20, -20, 0 };
        double xyz1[3] = { 20, 20, 5 };
        double mpp[3] = { .2, .2, .2 };
        occ_map::FloatVoxelMap fvm(xyz0, xyz1, mpp, 0);
        
        double ixyz[3];

        for (size_t k = 0; k < ret->no_points; ++k){
            if(fvm.isInMap(ret->points[k].xyz)){
                fvm.writeValue(ret->points[k].xyz,0.99);
            }
        }
        /*for (ixyz[2] = -5; ixyz[2] < 10; ixyz[2]+=.2) {
            for (ixyz[1] = -5; ixyz[1] < 10; ixyz[1]+=.2) {
                for (ixyz[0] = .5; ixyz[0] < 1; ixyz[0]+=.2) {
                    fvm.writeValue(ixyz,0.99);
                }
            }
        }

        double xyzO[3] = { 0, 0, 0 };
        double xyzR[3] = { 0, 5, 2 };
        for (double x = -5; x < 5; x += .5) {
            xyzR[0] = x;
            fvm.raytrace(xyzO, xyzR, 1, .3);
        }*/

        const occ_map_voxel_map_t * msg = fvm.get_voxel_map_t(bot_timestamp_now());
        lcm_t * lcm = lcm_create(NULL);
        occ_map_voxel_map_t_publish(lcm, "VOXEL_MAP",msg);

        //create and publish a voxel map 

        /*erlcm_xyz_point_list_t msg;
        msg.utime = bot_timestamp_now(); 
        msg.no_points = ret->no_points;
    
        msg.points = (erlcm_xyz_point_t *)calloc(msg.no_points, sizeof(erlcm_xyz_point_t));

        for (size_t k = 0; k < ret->no_points; ++k){
            msg.points[k].xyz[0] = ret->points[k].xyz[0];//cloud_p->points[k].x; 
            msg.points[k].xyz[1] = ret->points[k].xyz[1]; 
            msg.points[k].xyz[2] = ret->points[k].xyz[2]; 
        }

        //publish
        erlcm_xyz_point_list_t_publish(s->lcm, "PCL_XYZ_LIST", &msg);
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
    state->velodyne = velodyne_extractor_init(state->lcm);
    state->mainloop = g_main_loop_new( NULL, FALSE );  
  
    if (!state->mainloop) {
	printf("Couldn't create main loop\n");
	return -1;
    }

    //add lcm to mainloop 
    bot_glib_mainloop_attach_lcm (state->lcm);

    /* heart beat*/
    g_timeout_add (100, heartbeat_cb, state);

    //adding proper exiting 
    bot_signal_pipe_glib_quit_on_kill (state->mainloop);
    
    fprintf(stderr, "Starting Main Loop\n");

    ///////////////////////////////////////////////
    g_main_loop_run(state->mainloop);
  
    bot_glib_mainloop_detach_lcm(state->lcm);
}
