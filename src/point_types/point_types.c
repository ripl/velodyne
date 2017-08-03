#include "point_types.h"
#include <string.h>

void destroy_xyz_list(xyz_point_list_t *p_list){
    if(p_list != NULL){
        if(p_list->points !=NULL){
            free(p_list->points);
        }
        free(p_list);
    }
}

void destroy_xyz_array(xyz_point_array_t *p_array){
    if(p_array != NULL){
        if(p_array->points !=NULL){
            for(int i= 0; i < p_array->no_rows; i++){
                //for(int j = 0; j < p_array->no_columns; j++){
                if(p_array->points[i] != NULL){
                    free(p_array->points[i]);
                }
            }
        }
        free(p_array);
    }
}

xyz_point_array_t * create_xyz_array(int width, int height){
    xyz_point_array_t *new_array = (xyz_point_array_t *) calloc(1, sizeof(xyz_point_array_t));
    new_array->no_rows = height;
    new_array->no_columns = width;
    
    new_array->points = (xyz_point_t **) calloc(new_array->no_rows, sizeof(xyz_point_t *));
    for(int i=0; i < new_array->no_rows; i++){
        new_array->points[i] = (xyz_point_t *) calloc(new_array->no_columns, sizeof(xyz_point_t));
    }
    return new_array;
}

xyz_point_list_t * create_xyz_list(int size){
    xyz_point_list_t *new_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
    new_list->no_points = size;
    new_list->points = (xyz_point_t *) calloc(new_list->no_points, sizeof(xyz_point_t));
    return new_list;
}

void realloc_xyz_list(xyz_point_list_t *list, int size){
    list->no_points = size;
    list->points = (xyz_point_t *) realloc(list->points, size * sizeof(xyz_point_t));
}

xyz_point_list_t * copy_xyz_list(xyz_point_list_t *p_list){
    if(p_list != NULL){
        xyz_point_list_t *new_list = (xyz_point_list_t *) calloc(1, sizeof(xyz_point_list_t));
        if(p_list->points !=NULL){
            new_list->utime = p_list->utime;
            new_list->no_points = p_list->no_points;
            new_list->points = (xyz_point_t *) calloc(new_list->no_points, sizeof(xyz_point_t));
            memcpy(new_list->points , p_list->points, new_list->no_points *sizeof(xyz_point_t));            
        }
        return new_list;
    }
    return NULL;
}

void destroy_xyzr_list(xyzr_point_list_t *p_list){
    if(p_list != NULL){
        if(p_list->points !=NULL){
            free(p_list->points);
        }
        free(p_list);
    }
}

xyzr_point_list_t * create_xyzr_list(int size){
    xyzr_point_list_t *new_list = (xyzr_point_list_t *) calloc(1, sizeof(xyzr_point_list_t));
    new_list->no_points = size;
    new_list->points = (xyzr_point_t *) calloc(new_list->no_points, sizeof(xyzr_point_t));
    return new_list;
}

void realloc_xyzr_list(xyzr_point_list_t *list, int size){
    list->no_points = size;
    list->points = (xyzr_point_t *) realloc(list->points, size * sizeof(xyzr_point_t));
}

xyzr_point_list_t * copy_xyzr_list(xyzr_point_list_t *p_list){
    if(p_list != NULL){
        xyzr_point_list_t *new_list = (xyzr_point_list_t *) calloc(1, sizeof(xyzr_point_list_t));
        if(p_list->points !=NULL){
            new_list->utime = p_list->utime;
            new_list->no_points = p_list->no_points;
            new_list->points = (xyzr_point_t *) calloc(new_list->no_points, sizeof(xyzr_point_t));
            memcpy(new_list->points , p_list->points, new_list->no_points *sizeof(xyzr_point_t));            
        }
        return new_list;
    }
    return NULL;
}


