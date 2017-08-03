#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include <bot_core/bot_core.h>

#include "velodyne.h"


// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

/**
 * Macro for creating a gsl_vector_view object and data on the stack.
 * Use gslu macros to create vector elements on the stack with vector
 * views.  var will have subfields .data and .vector.  The advantage
 * is that elements created on the stack do not have to be manually
 * freed by the programmer, they automatically have limited scope
 */
#define GSLU_VECTOR_VIEW(var,i,...)                                     \
    struct {                                                            \
        double data[i];                                                 \
        gsl_vector vector;                                              \
    } var = {__VA_ARGS__};                                              \
        {   /* _view_ has local scope */                                \
            gsl_vector_view _view_ = gsl_vector_view_array (var.data, i); \
            var.vector = _view_.vector;                                 \
        }


// b = A*x
static inline int
gslu_mv (gsl_vector *b, const gsl_matrix *A, const gsl_vector *x)
{
    assert (b->size==A->size1 && A->size2==x->size);
    return gsl_blas_dgemv (CblasNoTrans, 1.0, A, x, 0.0, b);
}

// C = A*B
static inline int
gslu_mm (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    assert (C->size1==A->size1 && C->size2==B->size2 && A->size2==B->size1);
    return gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, C);
}


// return an upper bound on the # of samples in this message
//////int
//////velodyne_decoder_estimate_samples (velodyne_calib_t *calib, const void *_data, int datalen)
//////{
//////    return (datalen / 3) + 1;
//////}

int
velodyne_calib_precompute (velodyne_calib_t *calib);

void
velodyne_read_intrinsic_calibration_file(velodyne_calib_t* calib, char *db_xml_file_path);


// -----------------------------------------------------------------------------
// create the calibration structure
// -----------------------------------------------------------------------------
velodyne_calib_t *
velodyne_calib_create (velodyne_sensor_type_t sensor_type, char *db_xml_file_path)
{
    velodyne_calib_t *calib = calloc (1, sizeof (velodyne_calib_t));

    // get the number of lasers and sensor type
    if (sensor_type == VELODYNE_SENSOR_TYPE_VLP_16)
        calib->num_lasers = VELODYNE16_NUM_LASERS;
    else if (sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
        calib->num_lasers = VELODYNE32_NUM_LASERS;
    else if (sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1 ||
             sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S2)
        calib->num_lasers = VELODYNE64_NUM_LASERS;
    calib->sensor_type = sensor_type;

    calib->lasers = calloc (calib->num_lasers, sizeof (*calib->lasers));
    calib->physical2logical = calloc (calib->num_lasers, sizeof (*calib->physical2logical));
    calib->logical2physical = calloc (calib->num_lasers, sizeof (*calib->logical2physical));
    calib->va_sin = calloc (calib->num_lasers, sizeof (*calib->va_sin));
    calib->va_cos = calloc (calib->num_lasers, sizeof (*calib->va_cos));

    velodyne_read_intrinsic_calibration_file(calib, db_xml_file_path);

    calib->calibrated_intensity = calloc (calib->num_lasers, sizeof (int*));
    for(int i=0; i<(calib->num_lasers); i++)
        calib->calibrated_intensity[i] = calloc (256, sizeof(int));

    velodyne_calib_precompute (calib);

    bot_fasttrig_init();

    return calib;
}

void
velodyne_calib_free (velodyne_calib_t * calib) {
    free (calib->lasers);
    free (calib->physical2logical);
    free (calib->logical2physical);
    free (calib->va_sin);
    free (calib->va_cos);
    for(int i=0; i<(calib->num_lasers); i++)
        free(calib->calibrated_intensity[i]);
    free(calib->calibrated_intensity);
    free (calib);
}



velodyne_laser_return_collection_t *
velodyne_decode_data_packet_old(velodyne_calib_t* calib, const uint8_t *data,
                            int data_len, int64_t utime) {

    // Each data packet contains 1206 bytes consisting of
    //   12 x 100 byte packets: id (2 bytes), rotation (2 bytes), and 32 range (2 bytes) and intensity (1 byte) pairs
    //   4 bytes GPS timestamp
    //   2 bytes Unused

    assert (data_len == VELODYNE_DATA_PACKET_LEN);

    velodyne_laser_return_collection_t *lrc =
        velodyne_calloc_laser_return_collection (VELODYNE_NUM_LASER_RETURNS_PER_PACKET);

    // in a packet there are 12 firing blocks each 100 bytes with id, rotation, and 32 laser returns (range and intensity)
    int i_f = 0;    //firing index = 0..11
    int i_l = 0;    //laser index index = 0..31

    // TODO read status messages (including gps timestamp)

    // timestamp for collection based on first laser in packet
    // utime passed in represents last firing in packet
    // if we were all synced up time wise using GPS we could calculate using
    // timestamp parsed from packet which would be better, but now we are doing
    // timestamp sync in the driver and passing that utime in
    if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
        lrc->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(0, 0);
    else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_VLP_16) {
        lrc->utime = utime; // TODO: get timing table implementted for velodyne 16s
        fprintf (stdout, "Update timing table for VLP 16\n");
    }
    else
        lrc->utime = utime; // TODO: get timing table implementted for velodyne 64s

    int laser_offset = 0;

    // loop over each firing in this packet
    for (i_f = 0; i_f<VELODYNE_NUM_FIRING_PER_PACKET ; i_f++) {

        uint16_t start_id = VELODYNE_GET_START_IDENTIFIER(data, VELODYNE_DATA_FIRING_START(i_f));

        if (start_id == VELODYNE_UPPER_START_IDENTIFIER)
            //Upper block lasers are numbered 1-32 in the db.xml file
            //but they are 32-63 in the velodyne-uncalib.h file
            laser_offset = 0;
        else if (start_id == VELODYNE_LOWER_START_IDENTIFIER)
            laser_offset = 32;
        else
            fprintf (stderr, "ERROR: Unknown Velodyne start identifier %4x\n", start_id);


        // position of velodyne head, constant for all 32 measurements that follow

        //this is where the heading is obtained
        double ctheta = VELODYNE_GET_ROT_POS(data, VELODYNE_DATA_FIRING_START(i_f));


        // Check to see that we aren't missing any data based on the difference between reported headings
        // The magnitude of the difference between sequential angles shouldn't be greater than VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS radians
        static double ctheta_prev = 0;
        static double delta = 0;

        double this_delta = bot_mod2pi_ref(0, ctheta - ctheta_prev);

        if (fabs(this_delta) > VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS)
            fprintf (stderr, "Velodyne Decode: Firing angle  (%.2f deg) - previous angle (%.2f deg) = %.2f deg > %.2f deg!\n",
		     ctheta * 180/M_PI, ctheta_prev * 180/M_PI, this_delta * 180/M_PI,
		     VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS * 180/M_PI, ctheta, ctheta_prev);

        ctheta_prev = ctheta;

	if (ctheta >= 2*M_PI) {
            fprintf (stdout, "Velodyne Decode: Reported heading wraps around 2 PI, setting to zero.\n");
            ctheta = 0;
        }

        // cache the cos and sin of the constant heading measurement for the next 32 meas to follow
        double sin_ctheta, cos_ctheta;
        bot_fasttrig_sincos (ctheta, &sin_ctheta, &cos_ctheta);

        int64_t fire_start_utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(i_f, 0);

        // loop over each laser in this firing
        for (i_l = 0; i_l<VELODYNE_NUM_LASERS_PER_FIRING ; i_l++) {

            //fill this structure
            velodyne_laser_return_t *lr = &(lrc->laser_returns[i_f*VELODYNE_NUM_LASERS_PER_FIRING + i_l]);

            if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1) {
                // THIS SECTION IS TAKEN RIGHT FROM THE DARPA CODE FOR THE OLD VELODYNE 64s
                // NOT REALLY TESTED YET AFTER MODS TO FIT NEW INTERFACE

                // compensate for intershot yaw change
                if (i_l%4 == 0) { //yaw changes by SPIN_RATE*4 between the groups of four firings
                    ctheta += VELODYNE_SPIN_RATE * VELODYNE_INTRA_SHOT_USEC;
                    if (ctheta >= 2*M_PI)
                        ctheta = 0; //seems weird, shouldnt it roll over instead of going to zero? --NCB
                    bot_fasttrig_sincos (ctheta, &sin_ctheta, &cos_ctheta);
                }


                lr->physical = laser_offset + i_l;
                lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

                velodyne_laser_calib_t *params = &calib->lasers[lr->physical];

                lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
                lr->range     = (lr->raw_range + params->range_offset) * (1.0 + params->range_scale_offset);
                lr->ctheta    = ctheta;
                lr->theta     = bot_mod2pi_ref (M_PI, ctheta + params->rcf);
                lr->phi       = params->vcf;
                lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

                lr->ctheta = ctheta;

                double sin_theta, cos_theta;
                bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
                double sin_phi = calib->va_sin[lr->physical];
                double cos_phi = calib->va_cos[lr->physical];

                double Dxy;
                double Z;
                //Here following the MIT coordinate convention for the laser scanner.
                //MIT coordinate system ===> X forward, Y left and Z up.
                //Our MATLAB coordinate system ===> Y forward, X right and Z up.
                if(laser_offset == 0){
                    //Upper block
                    // The numbers and the equation to calculate Dxy and Z are taken from
                    // the presentation send by velodyne people: refer diagram.ppt.
                    Dxy = lr->range*cos_phi - 2.85*2.54*0.01*cos_phi;
                    Z = params->voffset + 2.8309*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 2.85*2.54*0.01*sin_phi;
                }
                else{
                    //Lower block
                    Dxy = lr->range*cos_phi - 1.8*2.54*0.01*cos_phi;
                    Z = params->voffset + 2.4464*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 1.8*2.54*0.01*sin_phi;
                }

                lr->xyz[0] = Dxy*cos_theta - params->hcf*cos_ctheta;
                lr->xyz[1] = Dxy*sin_theta + params->hcf*sin_ctheta;
                lr->xyz[2] = Z;


            }
            else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S2) {
                fprintf (stderr, "ERROR: decoding not implemented for VELODYNE_SENSOR_TYPE_HDL_64E_S2");
            }

            //laser firing is interleved

            //order of the lasers
            //from top to bottom
            //31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1, 30,
            //28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0

            //mapping from laser ind to top to down ind

            //{31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25,9, 24, 9, 24, 8 ,
            //23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0}

            //i.e. if((31-l_ind)%2  == 1) => ind = 31-l_ind

            //else if((31-l_ind)%2  == 0) => ind 31-l_ind - 15

            else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E) {

                // according to velodyne the 32E shouldn't need any calibration
                // beyond the vertical correction, so curently no other corrections
                // are applied

                lr->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(i_f, i_l);

                // compensate for intershot yaw change
                // at 100m this can be up to 1/4 of a meter of correction
                int64_t usec_since_first = lr->utime - fire_start_utime;
                double ctheta_yaw_cmp = ctheta + (VELODYNE_SPIN_RATE * usec_since_first);

                lr->physical = laser_offset + i_l;
                lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

                velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
                lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
                lr->range     = lr->raw_range;
                lr->ctheta    = ctheta;
                lr->theta     = ctheta_yaw_cmp;
                lr->phi       = params->vcf;
                lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

                 int order_id = 0;

                if((31-lr->physical)%2 == 1){
                    order_id = 31-lr->physical;
                }
                else{
                    order_id = 31-lr->physical -15;
                }

                double sin_theta, cos_theta;
                bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
                double sin_phi = calib->va_sin[lr->physical];
                double cos_phi = calib->va_cos[lr->physical];

                lr->xyz[0] = lr->range * cos_theta * cos_phi;
                lr->xyz[1] = -lr->range * sin_theta * cos_phi;
                lr->xyz[2] = lr->range * sin_phi;
	          }
            else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_VLP_16) {

            // according to velodyne the 32E shouldn't need any calibration
            // beyond the vertical correction, so curently no other corrections
            // are applied

            // TODO: TIMING OFFSET NOT YET IMPLEMENTED
            //lr->utime = utime + VELODYNE_16_LASER_FIRING_TIME_OFFSET(i_f, i_l);

            // compensate for intershot yaw change
            // at 100m this can be up to 1/4 of a meter of correction
            int64_t usec_since_first = lr->utime - fire_start_utime;
            double ctheta_yaw_cmp = ctheta + (VELODYNE_SPIN_RATE * usec_since_first);

            lr->physical = laser_offset + i_l;
            lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

            velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
            lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
            lr->range     = lr->raw_range;
            lr->ctheta    = ctheta;
            lr->theta     = ctheta_yaw_cmp;
            lr->phi       = params->vcf;
            lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

             int order_id = 0;

            if((31-lr->physical)%2 == 1){
                order_id = 31-lr->physical;
            }
            else{
                order_id = 31-lr->physical -15;
            }

            double sin_theta, cos_theta;
            bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
            double sin_phi = calib->va_sin[lr->physical];
            double cos_phi = calib->va_cos[lr->physical];

            lr->xyz[0] = lr->range * cos_theta * cos_phi;
            lr->xyz[1] = -lr->range * sin_theta * cos_phi;
            lr->xyz[2] = lr->range * sin_phi;
          }
        }
    }

    return lrc;

}

velodyne_laser_return_collection_t *
velodyne_decode_data_packet(velodyne_calib_t* calib, const uint8_t *data,
                            int data_len, int64_t utime) {

    // Each data packet contains 1206 bytes consisting of
    //   12 x 100 byte packets: id (2 bytes), rotation (2 bytes), and 32 range (2 bytes) and intensity (1 byte) pairs
    //   4 bytes GPS timestamp
    //   2 bytes Unused

    assert (data_len == VELODYNE_DATA_PACKET_LEN);

    velodyne_laser_return_collection_t *lrc =
        velodyne_calloc_laser_return_collection (VELODYNE_NUM_LASER_RETURNS_PER_PACKET);

    // in a packet there are 12 firing blocks each 100 bytes with id, rotation, and 32 laser returns (range and intensity)
    int i_f = 0;    //firing index = 0..11
    int i_l = 0;    //laser index index = 0..31

    // TODO read status messages (including gps timestamp)

    // timestamp for collection based on first laser in packet
    // utime passed in represents last firing in packet
    // if we were all synced up time wise using GPS we could calculate using
    // timestamp parsed from packet which would be better, but now we are doing
    // timestamp sync in the driver and passing that utime in
    if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
        lrc->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(0, 0);
    else
        lrc->utime = utime; // TODO: get timing table implementted for velodyne 64s

    int laser_offset = 0;

    // loop over each firing in this packet
    for (i_f = 0; i_f<VELODYNE_NUM_FIRING_PER_PACKET ; i_f++) {

        uint16_t start_id = VELODYNE_GET_START_IDENTIFIER(data, VELODYNE_DATA_FIRING_START(i_f));

        if (start_id == VELODYNE_UPPER_START_IDENTIFIER)
            //Upper block lasers are numbered 1-32 in the db.xml file
            //but they are 32-63 in the velodyne-uncalib.h file
            laser_offset = 0;
        else if (start_id == VELODYNE_LOWER_START_IDENTIFIER)
            laser_offset = 32;
        else
            fprintf (stderr, "ERROR: Unknown Velodyne start identifier %4x\n", start_id);


        // position of velodyne head, constant for all 32 measurements that follow

        //this is where the heading is obtained
        double ctheta = VELODYNE_GET_ROT_POS(data, VELODYNE_DATA_FIRING_START(i_f));


        // Check to see that we aren't missing any data based on the difference between reported headings
        // The magnitude of the difference between sequential angles shouldn't be greater than VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS radians
        static double ctheta_prev = 0;
        static double delta = 0;

        double this_delta = bot_mod2pi_ref(0, ctheta - ctheta_prev);

        if (fabs(this_delta) > VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS)
            fprintf (stderr, "Velodyne Decode: Firing angle  (%.2f deg) - previous angle (%.2f deg) = %.2f deg > %.2f deg!\n",
		     ctheta * 180/M_PI, ctheta_prev * 180/M_PI, this_delta * 180/M_PI,
		     VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS * 180/M_PI, ctheta, ctheta_prev);

        ctheta_prev = ctheta;

	       if (ctheta >= 2*M_PI) {
            fprintf (stdout, "Velodyne Decode: Reported heading wraps around 2 PI, setting to zero.\n");
            ctheta = 0;
        }

        // cache the cos and sin of the constant heading measurement for the next 32 meas to follow
        double sin_ctheta, cos_ctheta;
        bot_fasttrig_sincos (ctheta, &sin_ctheta, &cos_ctheta);

        int64_t fire_start_utime = utime;
        // TODO: Implement the same for 16 and 64
        if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E)
            fire_start_utime += VELODYNE_32_LASER_FIRING_TIME_OFFSET(i_f, 0);

        // loop over each laser in this firing
        for (i_l = 0; i_l<VELODYNE_NUM_LASERS_PER_FIRING ; i_l++) {

            //logical is bottom up
            int logical_ind = 31- velodyne_physical_to_logical (calib, laser_offset + i_l);
            //fill this structure
            velodyne_laser_return_t *lr = &(lrc->laser_returns[i_f*VELODYNE_NUM_LASERS_PER_FIRING + logical_ind]);

            if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S1) {
                // THIS SECTION IS TAKEN RIGHT FROM THE DARPA CODE FOR THE OLD VELODYNE 64s
                // NOT REALLY TESTED YET AFTER MODS TO FIT NEW INTERFACE

                // compensate for intershot yaw change
                if (i_l%4 == 0) { //yaw changes by SPIN_RATE*4 between the groups of four firings
                    ctheta += VELODYNE_SPIN_RATE * VELODYNE_INTRA_SHOT_USEC;
                    if (ctheta >= 2*M_PI)
                        ctheta = 0; //seems weird, shouldnt it roll over instead of going to zero? --NCB
                    bot_fasttrig_sincos (ctheta, &sin_ctheta, &cos_ctheta);
                }

                lr->physical = laser_offset + i_l;
                lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

                velodyne_laser_calib_t *params = &calib->lasers[lr->physical];

                lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
                lr->range     = (lr->raw_range + params->range_offset) * (1.0 + params->range_scale_offset);
                lr->ctheta    = ctheta;
                lr->theta     = bot_mod2pi_ref (M_PI, ctheta + params->rcf);
                lr->phi       = params->vcf;
                lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

                lr->ctheta = ctheta;

                double sin_theta, cos_theta;
                bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
                double sin_phi = calib->va_sin[lr->physical];
                double cos_phi = calib->va_cos[lr->physical];

                double Dxy;
                double Z;
                //Here following the MIT coordinate convention for the laser scanner.
                //MIT coordinate system ===> X forward, Y left and Z up.
                //Our MATLAB coordinate system ===> Y forward, X right and Z up.
                if(laser_offset == 0){
                    //Upper block
                    // The numbers and the equation to calculate Dxy and Z are taken from
                    // the presentation send by velodyne people: refer diagram.ppt.
                    Dxy = lr->range*cos_phi - 2.85*2.54*0.01*cos_phi;
                    Z = params->voffset + 2.8309*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 2.85*2.54*0.01*sin_phi;
                }
                else{
                    //Lower block
                    Dxy = lr->range*cos_phi - 1.8*2.54*0.01*cos_phi;
                    Z = params->voffset + 2.4464*2.54*0.01*sin_phi/cos_phi + lr->range*sin_phi - 1.8*2.54*0.01*sin_phi;
                }

                lr->xyz[0] = Dxy*cos_theta - params->hcf*cos_ctheta;
                lr->xyz[1] = Dxy*sin_theta + params->hcf*sin_ctheta;
                lr->xyz[2] = Z;


            }
            else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_64E_S2) {
                fprintf (stderr, "ERROR: decoding not implemented for VELODYNE_SENSOR_TYPE_HDL_64E_S2");
            }

            //laser firing is interleved

            //order of the lasers
            //from top to bottom
            //31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1, 30,
            //28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0

            //mapping from laser ind to top to down ind

            //{31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25,9, 24, 9, 24, 8 ,
            //23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0}

            //i.e. if((31-l_ind)%2  == 1) => ind = 31-l_ind

            //else if((31-l_ind)%2  == 0) => ind 31-l_ind - 15

            else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_HDL_32E) {

                // according to velodyne the 32E shouldn't need any calibration
                // beyond the vertical correction, so curently no other corrections
                // are applied

                lr->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(i_f, i_l);

                // compensate for intershot yaw change
                // at 100m this can be up to 1/4 of a meter of correction
                int64_t usec_since_first = lr->utime - fire_start_utime;
                double ctheta_yaw_cmp = ctheta + (VELODYNE_SPIN_RATE * usec_since_first);

                lr->physical = laser_offset + i_l;
                lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

                velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
                lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
                lr->range     = lr->raw_range;
                lr->ctheta    = ctheta;
                lr->theta     = ctheta_yaw_cmp;
                lr->phi       = params->vcf;
                lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

                /*int order_id = 0;

                if((31-lr->physical)%2 == 1){
                    order_id = 31-lr->physical;
                }
                else{
                    order_id = 31-lr->physical -15;
                    }*/

                double sin_theta, cos_theta;
                bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
                double sin_phi = calib->va_sin[lr->physical];
                double cos_phi = calib->va_cos[lr->physical];

                lr->xyz[0] = lr->range * cos_theta * cos_phi;
                lr->xyz[1] = -lr->range * sin_theta * cos_phi;
                lr->xyz[2] = lr->range * sin_phi;
	    }
      else if (calib->sensor_type == VELODYNE_SENSOR_TYPE_VLP_16) {

          // according to velodyne the 32E shouldn't need any calibration
          // beyond the vertical correction, so curently no other corrections
          // are applied

          // TODO: Time offset not implemented for VLP-16
          lr->utime = utime;
          //lr->utime = utime + VELODYNE_32_LASER_FIRING_TIME_OFFSET(i_f, i_l);

          // compensate for intershot yaw change
          // at 100m this can be up to 1/4 of a meter of correction
          int64_t usec_since_first = lr->utime - fire_start_utime;
          double ctheta_yaw_cmp = ctheta + (VELODYNE_SPIN_RATE * usec_since_first);

          lr->physical = laser_offset + i_l;
          lr->logical  = velodyne_physical_to_logical (calib, lr->physical);

          velodyne_laser_calib_t *params = &calib->lasers[lr->physical];
          lr->raw_range = VELODYNE_GET_RANGE(data, VELODYNE_DATA_LASER_START(i_f, i_l));
          lr->range     = lr->raw_range;
          lr->ctheta    = ctheta;
          lr->theta     = ctheta_yaw_cmp;
          lr->phi       = params->vcf;
          lr->intensity = VELODYNE_GET_INTENSITY(data, VELODYNE_DATA_LASER_START(i_f, i_l));

          /*int order_id = 0;

          if((31-lr->physical)%2 == 1){
              order_id = 31-lr->physical;
          }
          else{
              order_id = 31-lr->physical -15;
              }*/

          double sin_theta, cos_theta;
          bot_fasttrig_sincos (lr->theta, &sin_theta, &cos_theta);
          double sin_phi = calib->va_sin[lr->physical];
          double cos_phi = calib->va_cos[lr->physical];

          lr->xyz[0] = lr->range * cos_theta * cos_phi;
          lr->xyz[1] = -lr->range * sin_theta * cos_phi;
          lr->xyz[2] = lr->range * sin_phi;
}
        }
    }

    return lrc;

}

velodyne_laser_return_collection_t *
velodyne_calloc_laser_return_collection (int num_samples) {

    velodyne_laser_return_collection_t *lrc = calloc (1, sizeof (*lrc));
    lrc->num_lr = num_samples;
    lrc->laser_returns = calloc (lrc->num_lr, sizeof (*(lrc->laser_returns)));

    return lrc;
}

void
velodyne_free_laser_return_collection (velodyne_laser_return_collection_t *lrc) {

    free (lrc->laser_returns);
    free (lrc);
}


velodyne_laser_return_collector_t *
velodyne_laser_return_collector_create (uint8_t whole_scan, double start_angle, double end_angle) {

    velodyne_laser_return_collector_t *collector = calloc (1, sizeof (*collector));

    collector->whole_scan = whole_scan;
    collector->start_angle = start_angle;
    collector->end_angle = end_angle;
    collector->fov_angle = 0;
    collector->prev_angle = 0;
    collector->num_lr = 0;
    collector->laser_returns = g_array_new (FALSE, TRUE, sizeof (velodyne_laser_return_t));
    collector->state.utime = 0;
    collector->collection_ready = 0;
    collector->utime_collection = 0;
    collector->collecting = 0;

    return collector;
}

// clean up to start a new collection
void
velodyne_reset_collector (velodyne_laser_return_collector_t *collector) {

    g_array_free (collector->laser_returns, TRUE);
    collector->num_lr = 0;
    collector->laser_returns = g_array_new (FALSE, TRUE, sizeof (velodyne_laser_return_t));
    collector->utime_collection = 0;
    collector->collection_ready = 0;
    collector->first_laser_has_pose = 0;
}

void
velodyne_laser_return_collector_free (velodyne_laser_return_collector_t *collector) {

    g_array_free (collector->laser_returns, TRUE);
    free (collector);
}

uint8_t
in_collecting_range (double theta, double start, double end) {

    // two cases
    if (start <= end) { // no rollover
        if (theta <= end && theta >= start)
            return 1;
        else
            return 0;
    } else { // rollover at 2PI
        if (theta >= start || theta <= end)
            return 1;
        else
            return 0;
    }

}

void
motion_compensate (velodyne_laser_return_collector_t *collector,
                   velodyne_laser_return_collection_t *new_returns) {

    // data packet rate 1.8 Khz and pose rate max is about 0.1 Khz
    // so for each data packet we only need to find the first laser to current pose once
    double x_flp_cp[6] = {0}; //first laser pose to current pose
    double current_pose[6] = {collector->state.xyz[0], collector->state.xyz[1], collector->state.xyz[2],
                              collector->state.rph[0], collector->state.rph[1], collector->state.rph[2]};


    // Perform the equivalent of the ssc_tail2tail operation
    BotTrans current_pose_trans;
    BotTrans first_laser_pose_trans;
    BotTrans x_flp_cp_trans;

    double temp_quat[4];
    bot_roll_pitch_yaw_to_quat (collector->state.rph, temp_quat);

    bot_trans_set_from_quat_trans (&current_pose_trans, temp_quat, collector->state.xyz);

    double laser_pos[] = {collector->first_laser_pose[0], collector->first_laser_pose[1], collector->first_laser_pose[2]};
    double laser_rph[] = {collector->first_laser_pose[3], collector->first_laser_pose[4], collector->first_laser_pose[5]};

    bot_roll_pitch_yaw_to_quat (laser_rph, temp_quat);
    bot_trans_set_from_quat_trans (&first_laser_pose_trans, temp_quat, laser_pos);

    // Compute the tail (inverse) of the current_pose transformation
    bot_trans_invert (&current_pose_trans);

    bot_trans_invert_and_compose (&current_pose_trans, &first_laser_pose_trans, &x_flp_cp_trans);

    // Convert quaternion to rpy
    double x_flp_rph[3];
    bot_quat_to_roll_pitch_yaw (x_flp_cp_trans.rot_quat, x_flp_rph);

    memcpy (&(x_flp_cp[0]), x_flp_cp_trans.trans_vec, 3*sizeof(double));
    memcpy (&(x_flp_cp[3]), x_flp_rph, 3*sizeof(double));

    //ssc_tail2tail (x_flp_cp, NULL, collector->first_laser_pose, current_pose);

    // NOTE if you don't care about the difference in dt within a datapacket (approx 500 usec)
    // (for anything less than a car moving on the highway you probably don't)
    // you can save a lot of CPU time by motion compensating the entire datapacket at once
    // #if switches between this and motion compensating each individual laser return

#if 1 // compensate the entire data packet to at once based on timestamp of first laser in packet

    double dt = (new_returns->utime - collector->state.utime)/1e6;
    double x_flp_clp[6] = {0}; // pose at this laser firing (current pose + velocities*dt)
    x_flp_clp[0] = x_flp_cp[0] + dt*collector->state.xyz_dot[0];
    x_flp_clp[1] = x_flp_cp[1] + dt*collector->state.xyz_dot[1];
    x_flp_clp[2] = x_flp_cp[2] + dt*collector->state.xyz_dot[2];
    x_flp_clp[3] = x_flp_cp[3] + dt*collector->state.rph_dot[0];
    x_flp_clp[4] = x_flp_cp[4] + dt*collector->state.rph_dot[1];
    x_flp_clp[5] = x_flp_cp[5] + dt*collector->state.rph_dot[2];

    double R_clp_flp[9];
    double rph_flp_clp[3] = {x_flp_clp[3], x_flp_clp[4], x_flp_clp[5]};

    double quat_flp_clp[4];
    bot_roll_pitch_yaw_to_quat (rph_flp_clp, quat_flp_clp);
    bot_quat_to_matrix (quat_flp_clp, R_clp_flp);

    //so3_rotxyz (R_clp_flp, rph_flp_clp);
    gsl_matrix_view R_clp_flp_v = gsl_matrix_view_array (R_clp_flp, 3, 3);

    // stack all laser returns into matrix
    gsl_matrix *xyz_stacked = gsl_matrix_calloc (3, new_returns->num_lr);
    gsl_matrix *xyz_stacked_r = gsl_matrix_calloc (3, new_returns->num_lr);
    for (int i=0 ; i<(new_returns->num_lr) ; i++) {
        velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
        gsl_matrix_set (xyz_stacked, 0, i, lr->xyz[0]);
        gsl_matrix_set (xyz_stacked, 1, i, lr->xyz[1]);
        gsl_matrix_set (xyz_stacked, 2, i, lr->xyz[2]);
    }
    // rotate laser returns
    gslu_mm (xyz_stacked_r, &R_clp_flp_v.matrix, xyz_stacked);

    // translate and pull data out of matrix
    for (int i=0 ; i<(new_returns->num_lr) ; i++) {
        velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);
        lr->xyz[0] = gsl_matrix_get (xyz_stacked_r, 0, i) + x_flp_clp[0];
        lr->xyz[1] = gsl_matrix_get (xyz_stacked_r, 1, i) + x_flp_clp[0];
        lr->xyz[2] = gsl_matrix_get (xyz_stacked_r, 2, i) + x_flp_clp[0];
    }

    gsl_matrix_free (xyz_stacked);
    gsl_matrix_free (xyz_stacked_r);

#else // compensate each laser return based on its individual timestamp

    for (int i=0 ; i<(new_returns->num_lr) ; i++) {

        velodyne_laser_return_t *lr = &(new_returns->laser_returns[i]);

        // compensate laser
        double dt = (lr->utime - collector->state.utime)/1e6;
        double x_flp_clp[6] = {0}; // pose at this laser firing (current pose + velocities*dt)
        x_flp_clp[0] = x_flp_cp[0] + dt*collector->state.xyz_dot[0];
        x_flp_clp[1] = x_flp_cp[1] + dt*collector->state.xyz_dot[1];
        x_flp_clp[2] = x_flp_cp[2] + dt*collector->state.xyz_dot[2];
        x_flp_clp[3] = x_flp_cp[3] + dt*collector->state.rph_dot[0];
        x_flp_clp[4] = x_flp_cp[4] + dt*collector->state.rph_dot[1];
        x_flp_clp[5] = x_flp_cp[5] + dt*collector->state.rph_dot[2];

        double R_clp_flp[9];
        double rph_flp_clp[3] = {x_flp_clp[3], x_flp_clp[4], x_flp_clp[5]};

        double quat_flp_clp[4];
        bot_roll_pitch_yaw_to_quat (rph_flp_clp, quat_flp_clp);
        bot_quat_to_matrix (quat_flp_clp, R_clp_flp);

        //so3_rotxyz (R_clp_flp, rph_flp_clp);
        GSLU_VECTOR_VIEW (xyz, 3, {lr->xyz[0], lr->xyz[1], lr->xyz[2]});
        GSLU_VECTOR_VIEW (xyz_r, 3, {0});
        // rotate into first laser pose frame
        gsl_matrix_view R_clp_flp_v = gsl_matrix_view_array (R_clp_flp, 3, 3);
        gslu_mv (&xyz_r.vector, &R_clp_flp_v.matrix, &xyz.vector);
        // include tranlation between first laser pose and current laser pose
        lr->xyz[0] = gsl_vector_get (&xyz_r.vector, 0) + x_flp_clp[0];
        lr->xyz[1] = gsl_vector_get (&xyz_r.vector, 1) + x_flp_clp[1];
        lr->xyz[2] = gsl_vector_get (&xyz_r.vector, 2) + x_flp_clp[2];

        lr->motion_compensated = 1;
    }

#endif
}

velodyne_collector_push_return_t
velodyne_collector_push_laser_returns (velodyne_laser_return_collector_t *collector,
                                       velodyne_laser_return_collection_t *new_returns) {

    static double collector_ctheta_prev = 0;

    if (!collector->collecting) {

        // restart collecting again if we are collecting whole scans or if we have re-entered
        if (collector->whole_scan || in_collecting_range (new_returns->laser_returns[0].theta, collector->start_angle, collector->end_angle)) {

	    // reset collector
            velodyne_reset_collector (collector);
            if (collector->whole_scan) {
                collector->start_angle = new_returns->laser_returns[0].theta;
		collector->prev_angle = collector->start_angle;//new_returns->laser_returns[new_returns->num_lr-1].theta;
		collector->fov_angle = 0;
	    }
            collector->utime_first_laser = new_returns->laser_returns[0].utime;
	    collector_ctheta_prev = new_returns->laser_returns[0].theta;

            // do we have a pose for the start of this collection?
            if (collector->state.utime != 0) {
                double dt = (collector->state.utime - collector->utime_first_laser)/1e6;
                collector->first_laser_pose[0] = collector->state.xyz[0] + dt*collector->state.xyz_dot[0];
                collector->first_laser_pose[1] = collector->state.xyz[1] + dt*collector->state.xyz_dot[1];
                collector->first_laser_pose[2] = collector->state.xyz[2] + dt*collector->state.xyz_dot[2];
                collector->first_laser_pose[3] = collector->state.rph[0] + dt*collector->state.rph_dot[0];
                collector->first_laser_pose[4] = collector->state.rph[1] + dt*collector->state.rph_dot[1];
                collector->first_laser_pose[5] = collector->state.rph[2] + dt*collector->state.rph_dot[2];
                collector->first_laser_has_pose = 1;
            }

            collector->collecting = 1;
        }

    }

    if (collector->collecting) {


        // motion compensation
        // we have a pose to compensate into and the current pose isn't stale
        if (collector->first_laser_has_pose && abs(new_returns->utime - collector->state.utime) < 2e5) {

          // Compenstaed b/c there is an error that leads to significant heading offsets.
          //motion_compensate (collector, new_returns);
        }

        // push the new returns onto the collection
        for (int i=0 ; i<(new_returns->num_lr) ; i++) {
            g_array_append_val (collector->laser_returns, new_returns->laser_returns[i]);

            collector->num_lr++;

	    // Check whole scans for dropped readings.
	    double this_delta = bot_mod2pi_ref(0, new_returns->laser_returns[i].ctheta - collector_ctheta_prev);
	    if (fabs(this_delta) > VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS)
		fprintf (stderr, "Velodyne Collector: Firing angle  (%.2f deg) - previous angle (%.2f deg) = %.2f deg > %.2f deg!\n",
			 new_returns->laser_returns[i].ctheta * 180/M_PI, collector_ctheta_prev * 180/M_PI,
			 this_delta*180/M_PI, VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS * 180/M_PI);

	    collector_ctheta_prev =  new_returns->laser_returns[i].ctheta;
	}

        // check for collection finish
        if (collector->whole_scan) {

            // check if we have collected a whole scan
            double theta = new_returns->laser_returns[new_returns->num_lr-1].theta;

            double theta_max = new_returns->laser_returns[new_returns->num_lr-1].theta +
                VELODYNE_RADS_PER_PACKET;

	    double theta_delta = bot_mod2pi_ref (M_PI, theta - collector->prev_angle);
	    collector->fov_angle += theta_delta;
	    collector->prev_angle = theta;

	    if (collector->fov_angle > 2*M_PI) {

                collector->collecting = 0;
                collector->collection_ready = 1;

                return VELODYNE_COLLECTION_READY;
            }

        } else {

            if (!in_collecting_range (new_returns->laser_returns[new_returns->num_lr-1].theta,
                                      collector->start_angle, collector->end_angle)) {
                // done collecting, return
                collector->collecting = 0;
                collector->collection_ready = 1;

                return VELODYNE_COLLECTION_READY;
            }

        }
    }


    return VELODYNE_COLLECTION_PUSH_OK;
    //return VELODYNE_COLLECTION_PUSH_ERROR;
}

velodyne_collector_push_return_t
velodyne_collector_push_state (velodyne_laser_return_collector_t *collector,
                               velodyne_state_t state) {

    memcpy (&(collector->state), &state, sizeof (velodyne_state_t));

    return VELODYNE_COLLECTION_PUSH_OK;
}

velodyne_laser_return_collection_t *
velodyne_collector_pull_collection (velodyne_laser_return_collector_t *collector) {

    if (!collector->collection_ready) {
        return NULL;
    }

    velodyne_laser_return_collection_t *lrc = calloc (1, sizeof (*lrc));

    lrc->utime = collector->utime_first_laser;
    lrc->num_lr = collector->num_lr;
    lrc->laser_returns = calloc (lrc->num_lr, sizeof (*(lrc->laser_returns)));

    // copy the laser returns to the output
    for (int i=0; i<(lrc->num_lr); i++) {
        lrc->laser_returns[i] = g_array_index(collector->laser_returns, velodyne_laser_return_t, i);
    }

    // reset collector
    velodyne_reset_collector (collector);

    return lrc;
}


static velodyne_calib_t *__v;

static int
laser_phi_compare (const void *_a, const void *_b)
{
    int a = *((int*) _a);
    int b = *((int*) _b);

    if (__v->lasers[a].vcf < __v->lasers[b].vcf)
        return -1;
    return 1;
}

// NOT REENTRANT
int
velodyne_calib_precompute (velodyne_calib_t *calib)
{
    assert (!__v); // check for reentrancy...

    __v = calib;

    for (int i = 0; i < calib->num_lasers; i++)
        calib->logical2physical[i] = i;
    qsort (calib->logical2physical, calib->num_lasers, sizeof (int), laser_phi_compare);

    for (int logical = 0; logical < calib->num_lasers; logical++) {
        calib->physical2logical[calib->logical2physical[logical]] = logical;
    }

    for (int physical = 0; physical < calib->num_lasers; physical++) {
        bot_sincos (calib->lasers[physical].vcf, &calib->va_sin[physical], & calib->va_cos[physical]);
    }
    __v = NULL;

    return 0;
}

void
parse_current_item(xmlDocPtr doc, xmlNodePtr cur, velodyne_calib_t* calib, int index){

    //Get the node px
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"px"))){
            break;
        }
        cur = cur->next;
    }
    //printf("%s\n", cur->name);

    //Get all the data for current laser
    xmlChar *key;
    cur = cur->xmlChildrenNode;
    while(cur != NULL) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"id_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            index = atoi((char*)key);
            //printf("id: %s\n", key);
            xmlFree(key);
        }
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"rotCorrection_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].rcf = atof((char*)key)*M_PI/180;
            //printf("rotCorrection: %f\n", calib->lasers[index].rcf);
            xmlFree(key);
        }
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"vertCorrection_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].vcf = atof((char*)key)*M_PI/180;
            //printf("vertCorrection: %f\n", calib->lasers[index].vcf);
            xmlFree(key);
        }
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"distCorrection_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].range_offset = atof((char*)key)*0.01;
            //printf("distCorrection: %f\n", calib->lasers[index].range_offset);
            xmlFree(key);
        }
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"vertOffsetCorrection_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].voffset = atof((char*)key)*0.01;
            //printf("vertOffsetCorrection: %f\n", calib->lasers[index].voffset);
            xmlFree(key);
        }
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"horizOffsetCorrection_"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            calib->lasers[index].hcf = atof((char*)key)*0.01;
            //printf("horizOffsetCorrection: %f\n", calib->lasers[index].hcf);
            xmlFree(key);
        }
        cur = cur->next;
    }
}

void
velodyne_read_intrinsic_calibration_file(velodyne_calib_t* calib, char *db_xml_file_path){


    xmlDocPtr doc;
    xmlNodePtr cur;
    doc = xmlParseFile(db_xml_file_path);
    if (doc == NULL ) {
        fprintf(stderr,"Calibration file not parsed successfully. \n");
        return;
    }
    cur = xmlDocGetRootElement(doc);
    if (cur == NULL) {
        fprintf(stderr,"empty document\n");
        xmlFreeDoc(doc);
        return;
    }
    if (xmlStrcmp(cur->name, (const xmlChar *) "boost_serialization")) {
        fprintf(stderr,"document of the wrong type, root node != story");
        xmlFreeDoc(doc);
        return;
    }

    //Get the node DB
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"DB"))){
            break;
        }
        cur = cur->next;
    }
    //printf("%s\n", cur->name);

    // in the new db.xml they have a enabled node that tells you which of the 64
    // lasers are enabled

    //Get the node points_
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"points_"))){
            break;
        }
        cur = cur->next;
    }
    //printf("%s\n", cur->name);

    xmlNodePtr curPoints = cur;

    xmlChar *key;
    int numLasers;
    cur = cur->xmlChildrenNode;
    while (cur != NULL) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"count"))) {
            key = xmlNodeListGetString(doc, cur->xmlChildrenNode, 1);
            //printf("Count: %s\n", key);
            numLasers = atof((char*)key);
            xmlFree(key);
            break;
        }
        cur = cur->next;
    }

    //Loop over all the item of node points_
    cur = curPoints->xmlChildrenNode;
    int index = 0;
    while (cur != NULL &&  index < calib->num_lasers) {
        if ((!xmlStrcmp(cur->name, (const xmlChar *)"item"))) {
            //populate the calibration data in velodyne_calib_t struct.
            parse_current_item(doc, cur, calib, index) ;
            calib->lasers[index].range_scale_offset = 0;
            index++;
        }
        cur = cur->next;
    }

}

void
velodyne_read_calibrated_intensity(velodyne_calib_t* calib, char *db_xml_file_path){

    FILE *fptr = fopen(db_xml_file_path, "r");
    char temp;
    int i, j;
    for(i = 0; i<(calib->num_lasers); i++){
        for(j = 0; j < 256; j++){
            fscanf(fptr,"%d", &calib->calibrated_intensity[i][j]);
        }
        fscanf(fptr,"%c",&temp);
    }

}

void
velodyne_calib_dump (velodyne_calib_t *calib)
{
    printf ("velodyne_laser_calib_t[] = {\n");
    for (int i = 0; i < (calib->num_lasers); i++) {
        velodyne_laser_calib_t *params = &calib->lasers[i];
        printf ("   { %11.7f, %11.7f, %8.4f, %8.4f, %10.6f }, // laser %2d\n",
                params->rcf, params->vcf, params->hcf, params->range_offset, params->range_scale_offset, i+1);
    }
    printf ("};\n\n");
}
