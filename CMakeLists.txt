cmake_minimum_required(VERSION 3.0.2)
project(simple_loop_closure)

add_compile_options(-std=c++14 -O3)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_filters nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  thirdparty/nanoflann/include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/simple_loop_closure_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)