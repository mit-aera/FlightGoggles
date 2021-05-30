cmake_minimum_required(VERSION 2.8.3)
project(flightgoggles)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation)



add_message_files(
        FILES
        IRMarker.msg
        IRMarkerArray.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)


set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
        "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/."
        )


# Download FG executable to: ${CATKIN_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}

include_directories()


## Mark files for installation
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

