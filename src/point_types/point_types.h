#ifndef __point_types_h__
#define __point_types_h__

#include <stdio.h>
#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif
    typedef struct _xyz_point_t{
        double xyz[3]; 
    } xyz_point_t;

    typedef struct _xyz_point_list_t{
        int64_t utime;
        int no_points; 
        xyz_point_t *points;
    } xyz_point_list_t;
    
    typedef struct _xyz_point_array_t{
        int64_t utime;
        int no_rows; 
        int no_columns; 
        xyz_point_t **points;
    } xyz_point_array_t;
    
    void destroy_xyz_array(xyz_point_array_t *p_array);   

    void destroy_xyz_list(xyz_point_list_t *p_list); 

    xyz_point_list_t * copy_xyz_list(xyz_point_list_t *p_list);

    xyz_point_list_t * create_xyz_list(int size);
 
    void realloc_xyz_list(xyz_point_list_t *list, int size);

    typedef struct _xyzr_point_t{
        double xyz[3];
        double r; 
    } xyzr_point_t;

    typedef struct _xyzr_point_list_t{
        int64_t utime;
        int no_points; 
        xyzr_point_t *points;
    } xyzr_point_list_t;

    void destroy_xyzr_list(xyzr_point_list_t *p_list); 

    xyzr_point_list_t * copy_xyzr_list(xyzr_point_list_t *p_list);

    xyzr_point_list_t * create_xyzr_list(int size);
 
    void realloc_xyzr_list(xyzr_point_list_t *list, int size);

#ifdef __cplusplus
}
#endif

#endif
