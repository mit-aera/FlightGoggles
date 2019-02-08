cmake_minimum_required(VERSION 2.8.3)
project(flightgoggles_marker_visualizer)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation message_filters cv_bridge image_transport flightgoggles)

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES keyboard
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs message_filters cv_bridge image_transport flightgoggles
  DEPENDS OpenCV Boost
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${SDL_INCLUDE_DIR})

add_executable(flightgoggles_marker_visualizer src/flightgoggles_marker_visualizer_node.cpp)

target_link_libraries(flightgoggles_marker_visualizer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############


install(TARGETS flightgoggles_marker_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
