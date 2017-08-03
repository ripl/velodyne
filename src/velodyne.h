#ifndef __VELODYNE_H__
#define __VELODYNE_H__

#include <stdint.h>
#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include <bot_core/bot_core.h>

/**
 * @defgroup PerlsCommonVelodyne Velodyne Utility Functions
 * @brief Code to parse and motion compensate velodyne data packets
 * @ingroup PerlsCommon
 * @include: perls-common/velodyne.h
 *
 * @{
 */

#define VELODYNE_VLP_16_MODEL_STR       "VLP_16"
#define VELODYNE_HDL_32E_MODEL_STR      "HDL_32E"
#define VELODYNE_HDL_64E_S1_MODEL_STR   "HDL_64E_S1"
#define VELODYNE_HDL_64E_S2_MODEL_STR   "HDL_64E_S2"
#define VELODYNE64_NUM_LASERS            64
#define VELODYNE32_NUM_LASERS            32
#define VELODYNE16_NUM_LASERS            16

#define VELODYNE_RADIANS_PER_LSB 0.00017453293 // LSB/100 = degrees (pi/(180*100))
#define VELODYNE_METERS_PER_LSB  0.002
#define VELODYNE_SPIN_RATE       (10*360/1e6*(M_PI/180)) //10 Hz = 3600/1e6*DTOR rads/usec
#define VELODYNE_INTRA_SHOT_USEC (4) //4 microseconds
#define VELODYNE_RADS_PER_PACKET 0.0278
#define VELODYNE_MAX_DELTA_RADS_BETWEEN_FIRINGS 0.01 // The magnitude of the difference between the ctheta reported
                                                     // for sequential firings should not exceed this threshold.

#define VELODYNE_POSITION_PORT          8308    //!< UDP port for data packet
#define VELODYNE_DATA_PORT              2368    //!< UDP port for position packet
#define VELODYNE_DATA_PACKET_LEN        1206    //!< UDP length packet length in bytes
                                                //   100 bytes (id, rotation, and 32 range & intensity) x 12 + 6 byte GPS
#define VELODYNE_POSITION_PACKET_LEN    512     //!< UDP position packet length in bytes

#define VELODYNE_UPPER_START_IDENTIFIER 0xeeff
#define VELODYNE_LOWER_START_IDENTIFIER 0xddff

// defines for indicies and lengths in data packet
#define VELODYNE_DATA_TIMESTAMP_START         (1200)                       //!< Index of the start of the timestamp
#define VELODYNE_DATA_FIRING_START(i_f)       (100 * (i_f))                //!< Index of the start of firing given the firing ind (i_f=0...11)
#define VELODYNE_DATA_LASER_START(i_f, i_l)   (100 * (i_f) + 4 + 3 *(i_l)) //!< Index of the start of laser give the firing ind (i_f=0...11)
                                                                           // and laser ind (i_l=0..31)
#define VELODYNE_FIRING_BLOCK_LEN              100             //!< Length of a single firing block data
#define VELODYNE_NUM_FIRING_PER_PACKET        (12)             //!< Number of firings per packet
#define VELODYNE_NUM_LASERS_PER_FIRING        (32)             //!< Number of laser returns per firing
#define VELODYNE_NUM_LASER_RETURNS_PER_PACKET (12*32)          //!< Number of laser returns in a single data packet

// defines for location of quantities ind position packet
#define VELODYNE_POSITION_TIMESTAMP_START     (14 + 24 + 160)   //!< Packet index of the start the timestamp

/**
 * Get the timestamp in usec (since the top of the hour) from the data packet
 */
#define VELODYNE_GET_TIMESTAMP_USEC(data)  (data[VELODYNE_DATA_TIMESTAMP_START]  + \
                                            (data[VELODYNE_DATA_TIMESTAMP_START+1]<<8) + \
                                            (data[VELODYNE_DATA_TIMESTAMP_START+2]<<16) + \
                                            (data[VELODYNE_DATA_TIMESTAMP_START+3]<<24))

/**
 * Get start indentifier
 */
#define VELODYNE_GET_START_IDENTIFIER(data, start)    (data[start] +    \
                                                       (data[start+1]<<8))

/**
 * Get the rotational position from firing start
 */
#define VELODYNE_GET_ROT_POS(data, start)   ((data[start+2] + (data[start+3]<<8)) * VELODYNE_RADIANS_PER_LSB)

/**
 * Get the raw range from laser data start (VELODYNE_DATA_LASER_START)
 */
#define VELODYNE_GET_RANGE(data, start)     ((data[start] + (data[start+1]<<8)) * \
                                             VELODYNE_METERS_PER_LSB)

/**
 * Get the raw intensity from laser data start (VELODYNE_DATA_LASER_START)
 */
#define VELODYNE_GET_INTENSITY(data, start) (data[start+2] / 255.0)

/**
 * Get the offset, in microseconds, between a laser firing in the packet and
 * the last laser firing in the packet
 * given the firing index (i_f=0...11) and laser index (i_l=0..31)
 *
 * see velodyne 32 manual pg 24
 */
#define VELODYNE_32_LASER_FIRING_TIME_OFFSET(if, il) (-542.592 + (i_f*46.08) + (i_l*1.152))

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @brief enum of different Velodyne sensor models
     *
     */

    enum _velodyne_sensor_type_t {
        VELODYNE_SENSOR_TYPE_HDL_32E = 0,
        VELODYNE_SENSOR_TYPE_HDL_64E_S1 = 1,
        VELODYNE_SENSOR_TYPE_HDL_64E_S2 = 2,
        VELODYNE_SENSOR_TYPE_VLP_16 = 3,
    };
    typedef enum _velodyne_sensor_type_t velodyne_sensor_type_t;

    //we need a data structure that is a collection of planar lidar like (currently it is column ordered
    typedef struct _raw_scan_t {
        double  xyz[3];            //!< calibrated, projected into velodyne coordinate system
        double  range;             //!< corrected range
        double  theta;             //!< calibrated yaw angle **always [0, 2*PI)**
        //        double  phi;               //!< calibrated phi (pitch angle of laser)
        double  intensity;         //!< normalized intensity [0, 1]
        //int     logical;           //!< logical laser number (in order of increasing pitch)
        int64_t utime;
    } raw_scan_t;

    typedef struct _velodyne_planar_lidar_t {
        double pitch;
        int no_returns;
        //utimes change for each return - might need it
	double    rad0;
	double    radstep;
        raw_scan_t *ranges;
    } velodyne_planar_lidar_t;

    typedef struct velodyne_full_scan_t {
        int64_t utime;
        int no_lasers;
        velodyne_planar_lidar_t *lasers;
    } velodyne_full_scan_t;

    /**
     * @brief Velodyne range sample structure
     *
     */
    struct _velodyne_laser_return_t {
        double  xyz[3];            //!< calibrated, projected into velodyne coordinate system
        double  raw_range;         //!< raw return directly from sensor
        double  range;             //!< corrected range
        double  ctheta;            //!< yaw angle sensor head at time of sample, **always [0, 2*PI)**
        double  theta;             //!< calibrated yaw angle **always [0, 2*PI)**
        double  phi;               //!< calibrated phi (pitch angle of laser)
        double  intensity;         //!< normalized intensity [0, 1]
        int     physical;          //!< physical laser number (for velodyne 64: 0-31 lower, 32-63 upper)
        int     logical;           //!< logical laser number (in order of increasing pitch)
        uint8_t motion_compensated;//!<
        int64_t utime;             //!< utime of this laser return
    };

    typedef struct _velodyne_laser_return_t velodyne_laser_return_t;

    /**
     * @brief A collection of velodyne range sample structures
     *
     */

    struct _velodyne_laser_return_collection_t
    {
        uint32_t                  num_lr;           //!< number of laser returns
        velodyne_laser_return_t  *laser_returns;    //!< laser return data
        int64_t                   utime;            //!< base utime associated with this collection (FIRST LASER FIRING IN COLLECTION!)
        double                    pose[6];          //!< not used, supplied as convience for user (good place to cache your associated robot pose)
        uint8_t                   has_pose;         //!< not used, supplied as convience for user (good place to cache your associated robot pose)
    };

    typedef struct _velodyne_laser_return_collection_t velodyne_laser_return_collection_t;


    /**
     * @brief motion data for motion compensation
     *
     */
    struct _velodyne_state_t
    {
        double                    xyz[3];        //!< world frame pose
        double                    rph[3];        //!< wor3ld frame attitude
        double                    xyz_dot[3];    //!< world frame velocities
        double                    rph_dot[3];    //!< wrold frame rates
        int64_t                   utime;         //!< utime
    };

    typedef struct _velodyne_state_t velodyne_state_t;

    /**
     * @brief Structure used for laser return collection.
     * This structure is used to produce full or partial laser scans from multiple
     * data packets.
     */
    struct _velodyne_laser_return_collector_t {

        // setup params
        double     start_angle;         //!< setup: collect returns starting at this angle ()
        double     end_angle;           //!< setup: stop collecting returns at this angle
	double     fov_angle;           //!< current collector's FOV;
        double     prev_angle;          //!< angle of the last firing from previous scan
	uint8_t    whole_scan;          //!< setup: bool, collect whole scan, not segment
        uint8_t    collection_ready;    //!< flag if collecton is ready

        // internal
        int64_t             utime_first_laser;   //!< utime of first laser in this collection
        uint8_t             first_laser_has_pose;//!< flag if we have a pose for the first laser
        double              first_laser_pose[6]; //!< flag if we have a pose for the first laser
        velodyne_state_t    state;               //!< most recent sensor state (pose and motion)
        uint32_t            num_lr;              //!< number of laser returns
        GArray             *laser_returns;       //!< array of laser returns
        int64_t             utime_collection;    //!< utime that the collection is motion compensated to based on time of start angle
        uint8_t             collecting;          //!< flag if inside angle collecting data
    };

    typedef struct _velodyne_laser_return_collector_t  velodyne_laser_return_collector_t;

    /**
     * @brief enum collector push returns
     *
     */

    enum _velodyne_collector_push_return_t {
        VELODYNE_COLLECTION_READY = 0,
        VELODYNE_COLLECTION_PUSH_OK = 1,
        VELODYNE_COLLECTION_PUSH_ERROR = 2,
        VELODYNE_COLLECTION_READY_LOW = 3, //missing a bunch of returns
    };
    typedef enum _velodyne_collector_push_return_t velodyne_collector_push_return_t;

    /**
     * @brief Velodyne laser intrinsic calibration structure structure
     *
     */

    struct _velodyne_laser_calib_t {
        double rcf;                //!< rotational/yaw offset (radians)
        double vcf;                //!< vertical offset (radians)
        double hcf;                //!< horizontal off-axis offset (meters)
        double range_offset;       //!< range offset (meters)
        double voffset;            //!< vertical offset (meters)
        double range_scale_offset; //!< (scalar)
    };
    typedef struct _velodyne_laser_calib_t velodyne_laser_calib_t;
    /**
     * @brief Velodyne intrinsic calibration structure structure
     *
     * @details
     * physical laser numbers -> index based on firing order
     * logical laser numbers -> index based on increasing angle
     *
     *
     * @see velodyne_laser_calib
     */

    struct _velodyne_calib_t {
        velodyne_sensor_type_t sensor_type; //!< the type of sensor (32 or 64)
        int num_lasers;                     //!< the number of lasers for this sensor
        velodyne_laser_calib_t *lasers;     //!< laser calibrations (physical laser index)
        int *physical2logical;              //!< mapping physical laser index to logical laser index
        int *logical2physical;              //!< mapping logical laser index to physical laser index
        double *va_sin;                     //!< cached sin of vertical angle of laser (physical laser index)
        double *va_cos;                     //!< cached cos of vertical angle of laser (physical laser index)
        int **calibrated_intensity;         //!< intensity calibration lookup table
    };

    typedef struct _velodyne_calib_t velodyne_calib_t;

    /**
     * @brief create the velodyne_calib_t structure
     */
    velodyne_calib_t *
    velodyne_calib_create (velodyne_sensor_type_t sensor_type, char *db_xml_file_path);

    /**
     * @brief free the velodyne_calib_t structure
     */
    void
    velodyne_calib_free (velodyne_calib_t * calib);

    /**
     * @brief This function reads the intrinsic intensity calibration file
     *
     * @see velodyne_calib_t
     */
    void
    velodyne_read_calibrated_intensity(velodyne_calib_t* calib, char *db_xml_file_path);


    /**
     * @brief This function decodes a single data packet
     * returns an allocated velodyne_laser_return_collection_t structure
     * user needs to free when done with
     *
     * @see velodyne_free_laser_return_collection
     */
    velodyne_laser_return_collection_t *
    velodyne_decode_data_packet(velodyne_calib_t* calib, const uint8_t *data, int data_len, int64_t utime);


    /**
     * @brief Callocs a velodyne_laser_return_collection_t structure
     *
     */
    velodyne_laser_return_collection_t *
    velodyne_calloc_laser_return_collection (int num_samples);


    /**
     * @brief Frees a velodyne_laser_return_collection_t structure
     *
     */
    void
    velodyne_free_laser_return_collection (velodyne_laser_return_collection_t *lrc);


    /**
     * @brief create the velodyne_laser_return_collector_t structure
     *
     * @details set whole_scan=1 to collect 1 full revolution, if whole_scan=0 then use
     * start_angle, and end_angle to specify segment to collect
     */
    velodyne_laser_return_collector_t *
    velodyne_laser_return_collector_create (uint8_t whole_scan, double start_angle, double end_angle);

    /**
     * @brief free the velodyne_laser_return_collector_t structure
     */
    void
    velodyne_laser_return_collector_free (velodyne_laser_return_collector_t *collector);

    /**
     * @brief push new laser return data onto a collector
     *
     * @return returns velodyne_collector_push_return_t signaling if collection is
     * ready
     *
     */
    velodyne_collector_push_return_t
    velodyne_collector_push_laser_returns (velodyne_laser_return_collector_t *collector,
                                           velodyne_laser_return_collection_t *new_returns);

    /**
     * @brief push state data (pose and motion in world frame) onto a collector to be used for motion compensation.
     *
     * @details The user can push on world frame state, pose, velocities, and rates
     * Set unknown quantities to zero for no motion compensation
     *
     * @return returns velodyne_collector_push_return_t signaling if collection is
     * ready
     *
     */
    velodyne_collector_push_return_t
    velodyne_collector_push_state (velodyne_laser_return_collector_t *collector,
                                   velodyne_state_t state);

    /**
     * @brief pull a completed collection from the collector
     *
     * @returns a motion compensated collection of laser returns, returns null if not avaliable.
     *
     */
    velodyne_laser_return_collection_t *
    velodyne_collector_pull_collection (velodyne_laser_return_collector_t *collector);




    /**
     * @brief convert physical to logical laser index
     */
    static inline int
    velodyne_physical_to_logical (velodyne_calib_t *calib, int phys)
    {
        return calib->physical2logical[phys];
    }
    /**
     * @brief convert logical to physical laser index
     */
    static inline int
    velodyne_logical_to_physical (velodyne_calib_t *calib, int logical)
    {
        return calib->logical2physical[logical];
    }
    /**
     * @brief print the calibration parameters
     *
     * @see velodyne_calib_t
     */
    void
    velodyne_calib_dump (velodyne_calib_t *calib);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */


#endif // __PERLS_COMMON_VELODYNE_H__
