cmake_minimum_required(VERSION 3.0)
project(flightgoggles_ros_bridge)

add_compile_options(-std=c++11)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

################
# USER SETTINGS
################

# Turn this on to compile ROS bindings.
set(COMPILE_ROSCLIENT ON)

################
#set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
#
#set(CMAKE_CXX_STANDARD 11)
#
## Allow for optimized code with debug symbols.
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#
#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
#
## Fix cuda linking issues
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Find CUDA
#find_package(CUDA)

# Find OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Find OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Eigen
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Only compile ROS bindings if asked.
if (NOT COMPILE_ROSCLIENT)
    message(STATUS "Not building ROSClient since COMPILE_ROSCLIENT=OFF. Edit the top level CMakeLists.txt to compile ROS bindings.")
    set(COMPILE_ROSCLIENT OFF)
else ()
    message(STATUS "COMPILE_ROSCLIENT=ON, so building ROSClient.")

    # Init Catkin
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            tf2
            tf2_ros
            nav_msgs
            sensor_msgs
            geometry_msgs
            cv_bridge
            tf2_eigen
            image_transport
            flightgoggles
            )
    catkin_package(
            LIBRARIES simulator
            CATKIN_DEPENDS
            roscpp
            std_msgs
            tf2
            tf2_ros
            nav_msgs
            sensor_msgs
            geometry_msgs
            image_transport
            cv_bridge
            tf2_eigen
            flightgoggles

    )
endif ()

add_subdirectory(src)


