cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME velodyne)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

include_directories(${LCMTYPES_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)

find_package(OpenGL REQUIRED)
list(APPEND OPENGL_LIBRARIES GL)

#find_library (LIB_LAPACK lapack
#  DOC "Fullpath to lapack library.")

add_subdirectory(src/point_types)
add_subdirectory(src)

pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building velodyne renderer")
else()
    add_subdirectory(src/renderer)
endif()
