# Create a shared library libhello.so with a single source file
add_definitions(
  #-ggdb3
    -std=gnu99
    )

# Velodyne Library
add_library (velodyne SHARED velodyne.c)

set (REQUIRED_LIBS
    bot2-core
    libxml-2.0
    point_types
    gsl)

pods_use_pkg_config_packages (velodyne ${REQUIRED_LIBS})

set_target_properties(velodyne PROPERTIES SOVERSION 1)

# make the header public
pods_install_headers(velodyne.h DESTINATION velodyne)


# make the library public
pods_install_libraries(velodyne)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(velodyne
    LIBS -lvelodyne
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)


# Velodyne Driver
add_executable(velodyne-lcm main.c)

target_link_libraries (velodyne-lcm
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages (velodyne-lcm
    bot2-core
    bot2-param-client
    velodyne)

pods_install_executables (velodyne-lcm)

# Velodyne Receiver test
add_executable(velodyne-receiver test_handler.c)

target_link_libraries (velodyne-receiver
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages (velodyne-receiver
    bot2-core
    bot2-param-client
    hr-path-util
    velodyne)

pods_install_executables (velodyne-receiver)

# The function in test_ctheta.c reads Velodyne messages from
# an LCM log and looks for gaps in the reported heading that
# may be indicative of dropped packets

#add_executable(decode-velodyne-log test_ctheta.c)

#target_link_libraries (decode-velodyne-log
#    ${LCMTYPES_LIBS})

#pods_use_pkg_config_packages (decode-velodyne-log
#    velodyne)

#pods_install_executables (decode-velodyne-log)

add_definitions(
    -ggdb3
    -std=gnu99
    )

add_library(velodyne_extractor SHARED
    velodyne_extractor.c)

pods_install_headers(velodyne_extractor.h DESTINATION velodyne)

set (REQUIRED_LIBS
    velodyne
    bot2-core
    libxml-2.0
    lcmtypes_velodyne
    bot2-frames
    bot2-param-client
    hr-path-util
    gsl)

pods_install_libraries(velodyne_extractor)
pods_use_pkg_config_packages (velodyne_extractor ${REQUIRED_LIBS})

pods_install_pkg_config_file(velodyne_extractor
    CFLAGS
    LIBS -lvelodyne_extractor
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)

# Velodyne Driver
add_executable(velodyne-test test_velodyne_lib.c) #pp )

pods_use_pkg_config_packages (velodyne-test velodyne_extractor occ-map bot2-lcmgl-client)

pods_install_executables (velodyne-test)
