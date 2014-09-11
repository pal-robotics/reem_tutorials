cmake_minimum_required(VERSION 2.8.3)
project(reem_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  moveit_ros_planning_interface
  pal_camera_client
  pal_detection_msgs
  pal_face_database
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(progs
  reach_pose
  move_left_arm_joint_goal
  move_right_arm_torso_pose_goal
  look_to_point
  remote_monocular_image_visualization
  remote_stereo_image_visualization
  fullbody_person_2d_detection
  face_detection
  face_enrollment
  face_recognition
)

foreach(prog ${progs})
  set(target reem_tuto_${prog})
  add_executable(${target} src/${prog}.cpp)
  set_target_properties(${target} PROPERTIES OUTPUT_NAME ${prog})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${OpenCV_LIBS})
  list(APPEND targets ${target})
endforeach()


#############
## Install ##
#############

install(PROGRAMS
  scripts/reach_pose_moveit.py
  scripts/sonar_base_1.sh
  scripts/sonar_base_2.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
