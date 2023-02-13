cmake_minimum_required(VERSION 3.0.2)
project(visual_ndt_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_omp
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  tf_conversions 
)
find_package(Eigen3 REQUIRED)

# add_compile_options(-std=c++11)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS ndt_omp
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pointcloud_mapping
  src/pointcloud_mapping.cpp
)
add_dependencies(pointcloud_mapping
  ndt_omp
)
target_link_libraries(pointcloud_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ndt_omp
)

add_executable(ndt_optimizer
  src/ndt_optimizer.cpp
)
add_dependencies(ndt_optimizer
  ndt_omp
)
target_link_libraries(ndt_optimizer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ndt_omp
)

add_executable(ndt_odometry
  src/ndt_odometry.cpp
)
add_dependencies(ndt_odometry
  ndt_omp
)
target_link_libraries(ndt_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ndt_omp
)
