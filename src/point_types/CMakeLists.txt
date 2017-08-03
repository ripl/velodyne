# Create a shared library libhello.so with a single source file
add_definitions(
    #-ggdb3 
    -std=gnu99
)

pods_install_headers(point_types.h DESTINATION point_types)

add_library (point_types SHARED point_types.c)

set_target_properties(point_types PROPERTIES SOVERSION 1)

# make the library public
pods_install_libraries(point_types)

# uncomment these lines to link against another library via pkg-config
#set(REQUIRED_PACKAGES pkg_a pkg_b)
#pods_use_pkg_config_packages(hello ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(point_types
    CFLAGS
    LIBS -lpoint_types
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
