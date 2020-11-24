cmake_minimum_required(VERSION 2.8.3)
project(detection)

add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV 3.0 QUIET)
#find_package(octomap REQUIRED)
#include_directories(SYSTEM BEFORE ${OCTOMAP_INCLUDE_DIRS})
#link_directories(${OCTOMAP_LIBRARY_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Add support for C++11, supported in ROS Kinetic and newer
#SET(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
#find_package(tf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    # velodyne_msgs
    # octomap_ros
    # octomap_msgs cv_bridge
    message_filters
    tf
    tf_conversions
    eigen_conversions
    grid_map_core
    grid_map_ros
    grid_map_msgs
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
#  ${OCTOMAP_INCLUDE_DIRS}
)
catkin_package(
#  LIBRARIES octomap location
#  LIBRARIES octomap
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
    grid_map_core
    grid_map_ros
    grid_map_msgs
    eigen_conversions
    tf_conversions
    tf
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
#  velodyne_msgs
#  ndt_omp
)
set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PCL_LIBRARIES}
        )

add_executable(detection ${PROJECT_SOURCE_DIR}/src/detection.cpp)
target_link_libraries(detection ${catkin_LIBRARIES} ${LIBS})
include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(grid_map_test ${PROJECT_SOURCE_DIR}/src/grid_map_test.cpp)
target_link_libraries(grid_map_test ${catkin_LIBRARIES} ${LIBS})
include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(rawmapping ${PROJECT_SOURCE_DIR}/src/rawmapping.cpp)
target_link_libraries(rawmapping ${catkin_LIBRARIES} ${LIBS})
include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(grid_to_occupancy ${PROJECT_SOURCE_DIR}/src/grid_to_occupancy.cpp)
target_link_libraries(grid_to_occupancy ${catkin_LIBRARIES} ${LIBS})
include_directories(include ${catkin_INCLUDE_DIRS} )

#add_executable(listenpose ${PROJECT_SOURCE_DIR}/src/listenpose.cpp)
#target_link_libraries(listenpose ${catkin_LIBRARIES} ${LIBS})
#include_directories(include ${catkin_INCLUDE_DIRS} )

#add_executable(tf_pose_odom2camera ${PROJECT_SOURCE_DIR}/src/tf_pose_odom2camera.cpp)
#target_link_libraries(tf_pose_odom2camera ${catkin_LIBRARIES} ${LIBS})
#include_directories(include ${catkin_INCLUDE_DIRS} )

