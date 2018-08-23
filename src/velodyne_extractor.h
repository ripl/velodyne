#ifndef __velodyne_extractor_h__
#define __velodyne_extractor_h__

#include <velodyne/velodyne.h>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcmtypes/velodyne_t.h>
#include <lcmtypes/velodyne_list_t.h>
#include <point_types/point_types.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef void(on_velodyne_collection_update_t)(int64_t utime, void *user);

  typedef struct {
    on_velodyne_collection_update_t * callback_func;
    void * user;
  } on_velodyne_update_handler_t;

    typedef struct _extractor_state_t velodyne_extractor_state_t;
    struct _extractor_state_t {
        lcm_t *lcm;
        BotParam *param;
        BotFrames *frames;
        GMainLoop *mainloop;
        bot_core_pose_t *bot_pose_last;

        int64_t last_collector_utime;

        int have_data;
    
        velodyne_calib_t *calib;
        velodyne_laser_return_collector_t *collector;
    
        //this is protected by the mutex
        velodyne_laser_return_collection_t *lrc; 

        on_velodyne_update_handler_t *on_update;
    
        int64_t 	  last_velodyne_data_utime;
        int64_t           last_pose_utime;
        int64_t           last_data_return_time;
        GMutex *mutex;
        GMutex *mutex_lrc;
    };

    xyz_point_list_t * velodyne_extract_points(velodyne_extractor_state_t *self);
    
    velodyne_full_scan_t * velodyne_extract_scans(velodyne_extractor_state_t *self);
    
    xyz_point_list_t *velodyne_extract_points_frame_decimate(velodyne_extractor_state_t *self, char *frame, int decimate, double min_range, double max_range);

    velodyne_full_scan_t *velodyne_extract_new_scans(velodyne_extractor_state_t *self);

    void velodyne_full_scan_t_destroy(velodyne_full_scan_t *f_scan);

    xyz_point_list_t *velodyne_extract_points_frame(velodyne_extractor_state_t *self, char *frame);
    
  velodyne_extractor_state_t * velodyne_extractor_init(lcm_t *lcm, on_velodyne_collection_update_t *callback, void *user);

  velodyne_extractor_state_t * velodyne_extractor_init_full(lcm_t *lcm, uint8_t whole_scan, double start_angle, double end_angle, 
							    on_velodyne_collection_update_t *callback, void *user); 
    xyz_point_list_t *velodyne_extract_new_points_frame(velodyne_extractor_state_t *self, char *frame);
    
    xyz_point_list_t *velodyne_extract_new_points(velodyne_extractor_state_t *self);

    xyz_point_list_t *velodyne_extract_points_frame_decimate_compensation(velodyne_extractor_state_t *self, char *frame, int decimate, double min_range, double max_range);
    xyz_point_list_t *velodyne_extract_points_frame_compensation(velodyne_extractor_state_t *self, char *frame);

    xyzr_point_list_t *velodyne_extract_new_points_and_distance_frame(velodyne_extractor_state_t *self, char *frame);

    xyzr_point_list_t *velodyne_extract_points_and_distance_frame(velodyne_extractor_state_t *self, char *frame);

#ifdef __cplusplus
}
#endif

#endif
