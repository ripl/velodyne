cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME velodyne)
include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)

find_package(OpenGL REQUIRED)
list(APPEND OPENGL_LIBRARIES GL)

#find_library (LIB_LAPACK lapack
#  DOC "Fullpath to lapack library.")

add_subdirectory(src/point_types)
add_subdirectory(src)
