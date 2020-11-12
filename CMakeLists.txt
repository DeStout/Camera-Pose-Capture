cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_capture)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  moveit_core
  moveit_commander
  moveit_msgs
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  std_msgs
  joint_state_controller
  robot_state_publisher
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(capture_poses_cpp src/capture_poses.cpp)
target_link_libraries(capture_poses_cpp ${catkin_LIBRARIES})

add_executable(move_to_poses src/move_to_poses.cpp)
target_link_libraries(move_to_poses ${catkin_LIBRARIES})