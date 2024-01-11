cmake_minimum_required(VERSION 3.0.2)
project(kuka_arm_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  control_msgs
  sensor_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  moveit_visual_tools
  rviz_visual_tools
  message_generation
  tf2_ros
  tf2
)
find_package(Boost REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kuka_arm_manipulation
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${Eigen_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(trans_mat src/HomoTransMat.cpp)
add_executable(arm_traj_sampler src/arm_motion.cpp)
add_executable(robot_model_interface_node src/robot_model_interface.cpp)

## Add cmake target dependencies of the library
add_dependencies(trans_mat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_traj_sampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_model_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(trans_mat ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(arm_traj_sampler ${catkin_LIBRARIES})
target_link_libraries(robot_model_interface_node ${catkin_LIBRARIES})


