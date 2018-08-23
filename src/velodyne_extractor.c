#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>

#include <gsl/gsl_blas.h>

#include <hr_common/path_util.h>

#include "velodyne_extractor.h"

// convenience function to get the bot's position in the local frame
static int
frames_vehicle_pos_local (BotFrames *frames, double pos[3])
{
    double pos_body[3] = {0, 0, 0};
    return bot_frames_transform_vec (frames, "body", "local", pos_body, pos);
}

xyz_point_list_t *velodyne_extract_new_points(velodyne_extractor_state_t *self){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    if(self->last_data_return_time == self->lrc->utime){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    g_mutex_unlock (self->mutex_lrc);

    velodyne_extract_points(self);
}

xyz_point_list_t *velodyne_extract_points(velodyne_extractor_state_t *self){
    //, xyz_point_list_t *p_list){

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "local", self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);

        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, p_list->points[s].xyz);
    }

    //fprintf(stderr, "Size : %d\n", p_list->no_points);
    self->last_data_return_time = self->lrc->utime;
    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

void velodyne_full_scan_t_destroy(velodyne_full_scan_t *f_scan){
    if(f_scan == NULL)
        return;
    //there is this param also VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS
    for(int i=0; i < f_scan->no_lasers; i++){
        if(f_scan->lasers[i].ranges)
            free(f_scan->lasers[i].ranges);
    }
    free(f_scan->lasers);
    free(f_scan);
}

velodyne_full_scan_t *velodyne_extract_new_scans(velodyne_extractor_state_t *self){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    if(self->last_data_return_time == self->lrc->utime){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    g_mutex_unlock (self->mutex_lrc);

    velodyne_extract_scans(self);
}

velodyne_full_scan_t *velodyne_extract_scans(velodyne_extractor_state_t *self){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "local", self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    velodyne_full_scan_t *f_scan = (velodyne_full_scan_t *) calloc(1, sizeof(velodyne_full_scan_t));

    self->last_data_return_time = self->lrc->utime;

    //check if this has the correct no of points
    f_scan->no_lasers = VELODYNE32_NUM_LASERS;

    //this should be fixed if the scans are not dropped
    int no_scans_per_plane = ((double)self->lrc->num_lr)/ VELODYNE32_NUM_LASERS;
    f_scan->utime = self->lrc->utime;
    //vertically stacked planar scans
    f_scan->lasers =(velodyne_planar_lidar_t *) calloc(f_scan->no_lasers, sizeof(velodyne_planar_lidar_t));

    //there is this param also VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS
    for(int i=0; i < f_scan->no_lasers; i++){
        f_scan->lasers[i].no_returns = no_scans_per_plane;
        f_scan->lasers[i].ranges = (raw_scan_t *) calloc(no_scans_per_plane, sizeof(raw_scan_t));
    }

    int column_count = 0; //which column - horizontal
    int row_count = 0; //which row
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        //fill them
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);

        //ordered top to bottom
        //remainder
        //might be better to organize this better and do a memcpy
        raw_scan_t *rs = &f_scan->lasers[row_count].ranges[column_count];

        memcpy(rs->xyz,lr->xyz, sizeof(double) *3);
        rs->range = lr->range;
        rs->theta = lr->theta;
        rs->intensity = lr->intensity;
        rs->utime = lr->utime;

        row_count++;
        if(row_count == VELODYNE32_NUM_LASERS){
            row_count = 0;
            column_count++;
        }
    }

    g_mutex_unlock (self->mutex_lrc);

    return f_scan;
}

xyz_point_list_t *velodyne_extract_new_points_frame(velodyne_extractor_state_t *self, char *frame){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    if(self->last_data_return_time == self->lrc->utime){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    g_mutex_unlock (self->mutex_lrc);

    velodyne_extract_points_frame(self, frame);
}

xyz_point_list_t *velodyne_extract_points_frame(velodyne_extractor_state_t *self, char *frame){
    //extracts points in sensor frame

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  frame, self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);

        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, p_list->points[s].xyz);
    }

    //fprintf(stderr, "Size : %d\n", p_list->no_points);

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

xyzr_point_list_t *velodyne_extract_new_points_and_distance_frame(velodyne_extractor_state_t *self, char *frame){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    if(self->last_data_return_time == self->lrc->utime){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    g_mutex_unlock (self->mutex_lrc);

    velodyne_extract_points_and_distance_frame(self, frame);
}

xyzr_point_list_t *velodyne_extract_points_and_distance_frame(velodyne_extractor_state_t *self, char *frame){
    //extracts points in sensor frame

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  frame, self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyzr_point_list_t *p_list = (xyzr_point_list_t *) calloc(1, sizeof(xyzr_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyzr_point_t *) calloc(self->lrc->num_lr, sizeof(xyzr_point_t));

    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);

        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, p_list->points[s].xyz);
        p_list->points[s].r = lr->range;
    }

    //fprintf(stderr, "Size : %d\n", p_list->no_points);

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

xyz_point_list_t *velodyne_extract_points_frame_fov(velodyne_extractor_state_t *self, char *frame, double min_theta, double max_theta){
    //extracts points in sensor frame

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert



    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  frame, self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);

        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, p_list->points[s].xyz);
    }

    //fprintf(stderr, "Size : %d\n", p_list->no_points);

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

xyz_point_list_t *velodyne_extract_points_frame_decimate(velodyne_extractor_state_t *self, char *frame, int decimate, double min_range, double max_range){
    //extracts points in sensor frame

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert
    double sensor_to_body[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "body", self->lrc->utime,
                                                  sensor_to_body)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double body_to_frame[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "body",
                                                  frame, self->lrc->utime,
                                                  body_to_frame)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double sensor_to_frame[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  frame, self->lrc->utime,
                                                  sensor_to_frame)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    int p_c = 0;
    int column_count = 0;
    double point_b[3];
    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);
        column_count++;
        //fprintf(stderr,"Column count : %d => %d\n", column_count, s);
        if(column_count == VELODYNE32_NUM_LASERS){
            column_count = 0;
            s += decimate * VELODYNE32_NUM_LASERS -1;
        }

        if(lr->range > max_range || lr->range < min_range){
            continue;
        }


        if(lr->range < 1.4){
            bot_vector_affine_transform_3x4_3d (sensor_to_body, lr->xyz, point_b);//p_list->points[p_c].xyz);
            if(fabs(point_b[0]) < 0.4 && fabs(point_b[1]) < 0.6){
                continue;
            }
            bot_vector_affine_transform_3x4_3d (body_to_frame, point_b, p_list->points[p_c].xyz);
        }
        else{
            bot_vector_affine_transform_3x4_3d (sensor_to_frame, lr->xyz, p_list->points[p_c].xyz);
        }

        p_c++;
    }

    p_list->points = (xyz_point_t *) realloc(p_list->points, sizeof(xyz_point_t) * p_c);
    p_list->no_points = p_c;
    //fprintf(stderr, "Size : %d\n", p_list->no_points);

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

xyz_point_list_t *velodyne_extract_points_frame_decimate_compensation(velodyne_extractor_state_t *self, char *frame, int decimate, double min_range, double max_range){
    //extracts points in sensor frame

    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }
    //copy and convert
    double sensor_to_body[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "body", self->lrc->utime,
                                                  sensor_to_body)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double sensor_to_frame[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  frame, self->lrc->utime,
                                                  sensor_to_frame)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "local", self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double local_to_frame[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "local",
                                                  frame, self->lrc->utime,
                                                  local_to_frame)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    int p_c = 0;
    int column_count = 0;
    double point_b[3];
    int package_count = 0;
    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);
        //package_count =
        int c_pkg_count = (int ) floor(s / VELODYNE_NUM_LASER_RETURNS_PER_PACKET);

        //fprintf(stderr, "Package count : %d\n", package_count);

        //fprintf(stderr, "Firing : %d\n", s % VELODYNE_NUM_LASER_RETURNS_PER_PACKET );
        //if(s % VELODYNE_NUM_LASER_RETURNS_PER_PACKET == 0){
        if(package_count < c_pkg_count){
            package_count = c_pkg_count;
            //fprintf(stderr, "\dTime : %f\n", lr->utime/1.0e6);

            if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                          "local", lr->utime,
                                                          sensor_to_local)) {
                fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
                g_mutex_unlock (self->mutex_lrc);
                return NULL;
            }
        }

        column_count++;
        //fprintf(stderr,"Column count : %d => %d\n", column_count, s);
        if(column_count == VELODYNE32_NUM_LASERS){
            column_count = 0;
            s += decimate * VELODYNE32_NUM_LASERS -1;
        }

        if(lr->range > max_range || lr->range < min_range){
            continue;
        }


        if(lr->range < 1.4){
            bot_vector_affine_transform_3x4_3d (sensor_to_body, lr->xyz, point_b);
            if(fabs(point_b[0]) < 0.4 && fabs(point_b[1]) < 0.6){
                continue;
            }
            double point_l[3];
            bot_vector_affine_transform_3x4_3d (sensor_to_local,  lr->xyz, point_l);
            bot_vector_affine_transform_3x4_3d (local_to_frame,  point_l, p_list->points[p_c].xyz);
        }
        else{
            double point_l[3];
            bot_vector_affine_transform_3x4_3d (sensor_to_local,  lr->xyz, point_l);
            bot_vector_affine_transform_3x4_3d (local_to_frame,  point_l, p_list->points[p_c].xyz);
        }

        p_c++;
    }

    p_list->points = (xyz_point_t *) realloc(p_list->points, sizeof(xyz_point_t) * p_c);
    p_list->no_points = p_c;
    //fprintf(stderr, "Size : %d\n", p_list->no_points);

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}

xyz_point_list_t *velodyne_extract_points_frame_compensation(velodyne_extractor_state_t *self, char *frame){
    g_mutex_lock (self->mutex_lrc);

    if(self->lrc ==NULL){
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double sensor_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                  "local", self->lrc->utime,
                                                  sensor_to_local)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    double local_to_frame[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "local",
                                                  frame, self->lrc->utime,
                                                  local_to_frame)) {
        fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
        g_mutex_unlock (self->mutex_lrc);
        return NULL;
    }

    xyz_point_list_t *p_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    p_list->no_points = self->lrc->num_lr;
    p_list->utime = self->lrc->utime;
    p_list->points = (xyz_point_t *) calloc(self->lrc->num_lr, sizeof(xyz_point_t));

    int p_c = 0;
    int column_count = 0;
    double point_b[3];
    int package_count = 0;
    //fprintf(stderr,"Processing\n");
    for (unsigned int s = 0; s < self->lrc->num_lr; s++) {
        velodyne_laser_return_t *lr = &(self->lrc->laser_returns[s]);
        int c_pkg_count = (int ) floor(s / VELODYNE_NUM_LASER_RETURNS_PER_PACKET);

        if(package_count < c_pkg_count){
            package_count = c_pkg_count;

            if (!bot_frames_get_trans_mat_3x4_with_utime (self->frames, "VELODYNE",
                                                          "local", lr->utime,
                                                          sensor_to_local)) {
                fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
                g_mutex_unlock (self->mutex_lrc);
                return NULL;
            }
        }

        double point_l[3];
        bot_vector_affine_transform_3x4_3d (sensor_to_local,  lr->xyz, point_l);
        bot_vector_affine_transform_3x4_3d (local_to_frame,  point_l, p_list->points[s].xyz);
    }

    g_mutex_unlock (self->mutex_lrc);

    return p_list;
}


static int
process_velodyne (const velodyne_t *v, velodyne_extractor_state_t *self)
{
    g_assert(self);

    int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose

    // Is this a scan packet?
    if (v->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {

        velodyne_laser_return_collection_t *lrc =
            velodyne_decode_data_packet(self->calib, v->data, v->datalen, v->utime);

        int ret = velodyne_collector_push_laser_returns (self->collector, lrc);

        velodyne_free_laser_return_collection (lrc);

        if (VELODYNE_COLLECTION_READY == ret) {
            g_mutex_lock (self->mutex_lrc);
            if(self->lrc != NULL){
                velodyne_free_laser_return_collection (self->lrc);
            }
            //velodyne_laser_return_collection_t *lrc =
            self->lrc = velodyne_collector_pull_collection (self->collector);
            g_mutex_unlock (self->mutex_lrc);

	    //do the callback if ready
	    if(self->on_update){
	      self->on_update->callback_func(self->lrc->utime , self->on_update->user);
	    }

            //starting a new collection
            do_push_motion = 1;
        }
        else if(VELODYNE_COLLECTION_READY_LOW == ret) {
            fprintf(stderr,"Low packet - ignoring");

            velodyne_laser_return_collection_t *lrc =
                velodyne_collector_pull_collection (self->collector);

            velodyne_free_laser_return_collection (lrc);
        }
    }

    // Update the Velodyne's state information (pos, rpy, linear/angular velocity)
    /*if (do_push_motion) {

        if (!self->bot_pose_last)
            return 0;

        // push new motion onto collector
        velodyne_velodyne_extractor_state_t state;

        state.utime = v->utime;

        // find sensor pose in local/world frame

        BotTrans velodyne_to_local;
        bot_frames_get_trans_with_utime (self->frames, "VELODYNE", "local", v->utime, &velodyne_to_local);

        memcpy (state.xyz, velodyne_to_local.trans_vec, 3*sizeof(double));
        bot_quat_to_roll_pitch_yaw (velodyne_to_local.rot_quat, state.rph);

        // Compute translational velocity
        //
        // v_velodyne = v_bot + r x w
        BotTrans velodyne_to_body;
        bot_frames_get_trans (self->frames, "VELODYNE", "body", &velodyne_to_body);

        double v_velodyne[3];
        double r_body_to_velodyne_local[3];
        bot_quat_rotate_to (self->bot_pose_last->orientation, velodyne_to_body.trans_vec, r_body_to_velodyne_local);

        // r x w
        double vel_rot[3];
        bot_vector_cross_3d (r_body_to_velodyne_local, self->bot_pose_last->rotation_rate, vel_rot);

        bot_vector_add_3d (state.xyz_dot, vel_rot, self->bot_pose_last->vel);


        // Compute angular rotation rate
        memcpy (state.rph_dot, self->bot_pose_last->rotation_rate, 3*sizeof(double));

        do_push_motion = 0;
        }  */

    return 1;
}

static void
on_bot_pose (const lcm_recv_buf_t *buf, const char *channel,
             const bot_core_pose_t *msg, void *user) {

    velodyne_extractor_state_t *self =  (velodyne_extractor_state_t *) user;

    g_mutex_lock (self->mutex);

    if (self->bot_pose_last)
        bot_core_pose_t_destroy (self->bot_pose_last);
    self->bot_pose_last = bot_core_pose_t_copy (msg);

    g_mutex_unlock (self->mutex);
}



static void
on_velodyne_list (const lcm_recv_buf_t *rbuf, const char *channel,
		  const velodyne_list_t *msg, void *user)
{
    velodyne_extractor_state_t *self = (velodyne_extractor_state_t *)user;

    //fprintf(stderr,".");

    static int64_t last_redraw_utime = 0;
    int64_t now = bot_timestamp_now();

    for (int i=0; i < msg->num_packets; i++)
	process_velodyne (&(msg->packets[i]), self);

    return;
}

velodyne_extractor_state_t * velodyne_extractor_init(lcm_t *lcm,
						     on_velodyne_collection_update_t *callback, void *user){
  velodyne_extractor_init_full(lcm, 1, 0 ,2 * M_PI, callback, user);
}

velodyne_extractor_state_t * velodyne_extractor_init_full(lcm_t *lcm, uint8_t whole_scan, double start_angle, double end_angle,
							  on_velodyne_collection_update_t *callback, void *user){
    velodyne_extractor_state_t *state = (velodyne_extractor_state_t*) calloc(1, sizeof(velodyne_extractor_state_t));

    state->lcm = lcm;
    state->lrc = NULL;
    bot_glib_mainloop_attach_lcm (state->lcm);
    state->param = bot_param_new_from_server(state->lcm, 1);

    state->on_update = NULL;
    if(callback != NULL){
      state->on_update = (on_velodyne_update_handler_t *) calloc(1, sizeof(on_velodyne_update_handler_t));
      state->on_update->callback_func = callback;
      state->on_update->user = user;
    }

    state->frames = bot_frames_get_global (state->lcm, state->param);

    char key[256] = {'\0'};

    snprintf (key, sizeof(key), "%s.channel", "calibration.velodyne");
    char *lcm_channel = bot_param_get_str_or_fail (state->param, key);

    char lcm_channel_list[256];
    snprintf (lcm_channel_list, sizeof(lcm_channel_list), "%s_LIST", lcm_channel);

    char *velodyne_model = bot_param_get_str_or_fail (state->param, "calibration.velodyne.model");
    char *calib_file = bot_param_get_str_or_fail (state->param, "calibration.velodyne.intrinsic_calib_file");

    char calib_file_path[2048];

    sprintf(calib_file_path, "%s/%s", getConfigPath(), calib_file);

    if (0 == strcmp (velodyne_model, VELODYNE_HDL_32E_MODEL_STR))
        state->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_32E, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S1_MODEL_STR))
        state->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S1, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_HDL_64E_S2_MODEL_STR))
        state->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_HDL_64E_S2, calib_file_path);
    else if (0 == strcmp (velodyne_model, VELODYNE_VLP_16_MODEL_STR))
        state->calib = velodyne_calib_create (VELODYNE_SENSOR_TYPE_VLP_16, calib_file_path);
    else
        fprintf (stderr, "ERROR: Unknown Velodyne model \'%s\'", velodyne_model);

    free (velodyne_model);
    free (calib_file);

    state->collector = velodyne_laser_return_collector_create (whole_scan, start_angle, end_angle); // full scan

    state->mutex = g_mutex_new ();
    state->mutex_lrc = g_mutex_new ();

    velodyne_list_t_subscribe (state->lcm, lcm_channel_list, on_velodyne_list, state);

    // Subscribe to the POSE message
    bot_core_pose_t_subscribe (state->lcm, "POSE", on_bot_pose, state);

    free (lcm_channel);

    return state;
}
