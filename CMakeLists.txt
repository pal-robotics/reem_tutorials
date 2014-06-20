cmake_minimum_required(VERSION 2.8.3)
project(reem_tutorials)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  cv_bridge
  joint_state_publisher
  moveit_msgs
  moveit_ros_planning_interface
  pal_detection_msgs
  robot_state_publisher
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS control_msgs cv_bridge joint_state_publisher moveit_msgs moveit_ros_planning_interface pal_detection_msgs robot_state_publisher roscpp sensor_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(move_left_arm_joint_goal             src/move_left_arm_joint_goal.cpp)
target_link_libraries(move_left_arm_joint_goal ${catkin_LIBRARIES})

add_executable(move_right_arm_torso_pose_goal       src/move_right_arm_torso_pose_goal.cpp)
target_link_libraries(move_right_arm_torso_pose_goal ${catkin_LIBRARIES})

add_executable(look_to_point                        src/look_to_point.cpp)
target_link_libraries(look_to_point ${catkin_LIBRARIES})

add_executable(face_detection src/face_detection.cpp)
target_link_libraries(face_detection ${catkin_LIBRARIES})
add_dependencies(face_detection pal_detection_msgs_generate_messages_cpp)

install(TARGETS move_left_arm_joint_goal
                move_right_arm_torso_pose_goal
                look_to_point
                face_detection
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

