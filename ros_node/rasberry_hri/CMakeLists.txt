cmake_minimum_required(VERSION 2.8.3)
project(rasberry_hri)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #message_generation
  #geometry_msgs
  #people_msgs
  roscpp
  rospy
  openpose_ros
  bayes_people_tracker
  actionlib
  qsr_lib
  std_msgs
  std_srvs
  actionlib_msgs
  move_base_msgs
  geometry_msgs
  marvelmind_nav
  rasberry_people_perception
  rasberry_navigation
  sensor_msgs
  #strands_navigation_msgs
  tf
  #topological_navigation
)

 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Log.msg
  Criterium.msg
  Pose.msg
  Classification.msg

  Joint.msg
#  Joints.msg
  Action.msg
  Command.msg
)

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
   std_msgs  # Or other packages containing msgs
   sensor_msgs
   geometry_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rasberry_hri
   CATKIN_DEPENDS std_msgs
               actionlib
               actionlib_msgs
               move_base_msgs
               sensor_msgs
               geometry_msgs

#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  scripts/action_recognition_node
  scripts/robot_control_node
  scripts/scheduler_node
  scripts/picker_mover_node
  scripts/experimenter_node
#  scripts/skeleton_extractor_node
#  scripts/skeleton_converter_node
#  scripts/skeleton_stabilizer_node
#  scripts/skeleton_comparator_node

#  scripts/skeleton_logger_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ${USE_SOURCE_PERMISSIONS}
)


install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
