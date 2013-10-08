reem_tutorials
==============

Basic tutorials to get familiar with the REEM robot in a simulated environment.

With this package you will be able to test the movement of the arms and the head.

Having the REEM Simulation installed (you can follow this instructions:
https://docs.google.com/document/d/1bLLZQEKBLMJMEl6Npb8MRARDOnQFvQUXMhHtsjF-BxA/edit?usp=sharing ) you can use this package doing:

Launch reem model in gazebo:
roslaunch reem_gazebo reem_empty_world.launch

Launch rviz (if things look wrong maybe you need to change to /base_link from /map the fixed frame):
rosrun rviz rviz -d `rospack find reem_tutorials`/config/reem.vcg

Load some params:
rosparam load `rospack find reem_tutorials`/config/reem_poses.yaml

Try to reach some poses:
rosrun reem_tutorials reach_pose init
rosrun reem_tutorials reach_pose surrender
rosrun reem_tutorials reach_pose home


Launch arm navigation:
roslaunch reem_arm_navigation reem_arm_navigation.launch

Try to reach some poses:
rosrun reem_tutorials reach_pose hands_up
rosrun reem_tutorials reach_pose raise_left_hand
rosrun reem_tutorials move_left_arm_joint_goal



Another demo, launch a different reem world:
roslaunch reem_tutorials reem_look_to_point_world.launch
rosparam load `rospack find reem_tutorials`/config/reem_poses.yaml
rosrun reem_tutorials reach_pose home

Launch the look to point demo, click where you want the robot to look at:
rosrun reem_tutorials look_to_point


