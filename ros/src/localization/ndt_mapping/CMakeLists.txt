cmake_minimum_required(VERSION 2.8.3)
project(ndt_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(CUDA)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_mapping
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions roscpp rospy std_msgs tf sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
if (CUDA_FOUND)
	add_definitions(-DCUDA_FOUND)
	include_directories(${CUDA_INCLUDE_DIRS})
endif ()
link_directories(lib) # link compiled ndt_gpu .so
add_executable(${PROJECT_NAME}_node src/ndt_mapping_node.cpp src/NDTMapping.cpp src/NDTMapping.h)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
if (CUDA_FOUND)
	target_link_libraries(${PROJECT_NAME}_node ndt_gpu)
endif ()




