#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 11:15:46 2013

@author: Sam
"""

import sys
import rospy
import actionlib

from moveit_msgs.msg import MoveGroupGoal, MoveGroupAction, JointConstraint, Constraints, MoveItErrorCodes
# Dictionary to get the string representing the error code, very handy
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def createJointConstraints(pose_from_params, tolerances=0.1):
    """Create a JointConstraints message with its joint names and positions with some default tolerances
    @param pose_from_params dictionary with names of the joints and it's values
    @param tolerances the tolerance in radians for the joint positions, defaults to 0.1
    @return moveit_msgs/JointConstraints[] message with the joint names and positions"""
    joint_constraints = []
    for joint in pose_from_params:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint
        joint_constraint.position = pose_from_params[joint]
        joint_constraint.tolerance_above = tolerances
        joint_constraint.tolerance_below = tolerances
        joint_constraint.weight = 1.0
        joint_constraints.append(joint_constraint)
    return joint_constraints

def createMoveGroupGoal(move_group="both_arms", pose_from_params={}, tolerances=0.1):
    """Create a moveit_msgs/MoveGroupGoal with the pose passed in pose_from_params with some default values
    @param move_group the move_group from MoveIt! which needs to execute the movement of the joints, defaults to both_arms
    @param pose_from_params the pose as imported from the param server in the reem_tutorials containing joints and positions
    @param tolerances tolerances to all the joints above and below, defaults to 0.1
    @return moveit_msgs/MoveGroupGoal with the goal contained in the pose_from_params
    """
    mg_g = MoveGroupGoal()
    mg_g.request.group_name = move_group
    mg_g.request.allowed_planning_time = 5.0
    mg_g.request.num_planning_attempts = 3
    c = Constraints()
    joint_constraints = createJointConstraints(pose_params, tolerances=0.1)
    c.joint_constraints.extend(joint_constraints)
    mg_g.request.goal_constraints.append(c)
    mg_g.planning_options.plan_only = False
    # Next planning options are... optional
    mg_g.planning_options.replan = True
    mg_g.planning_options.replan_attempts = 3
    mg_g.planning_options.replan_delay = 0.3
    return mg_g
    
def showHelp(reason):
    if reason == "no_args":
        print "Usage:"
        print sys.argv[0], "name_of_pose"
    elif reason == "no_params":
        rospy.logerr("No params under '/reach_pose/poses' global name.")
        rospy.loginfo("Maybe you forgot to load poses with:\nrosparam load `rospack find reem_tutorials`/config/reem_poses.yaml")
    elif reason == "no_pose":
        rospy.logerr("Pose: " + str(name_of_movement) + " isn't loaded in rosparam server.")
        rospy.loginfo("Maybe you forgot to load poses with:\nrosparam load `rospack find reem_tutorials`/config/reem_poses.yaml")
    exit(-1)

if __name__=='__main__':
    # Check arguments
    if len(sys.argv) != 2:
        showHelp("no_args")
    name_of_movement = str(sys.argv[1])
    
    # Check if params are loaded
    rospy.init_node('reach_pose_moveit', anonymous=True)
    global_name = '/reach_pose/poses'
    if not rospy.has_param(global_name):
        showHelp("no_params") 

    # Load joints configs of pose by name
    if not rospy.has_param(global_name + '/' + name_of_movement):
        showHelp("no_pose")
    pose_params = rospy.get_param(global_name + '/' + name_of_movement)
    
    # Connect to action server
    rospy.loginfo("Connecting to move_group action server")
    client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    client.wait_for_server()
    
    # Create goal for action server
    move_group_goal = createMoveGroupGoal("both_arms", pose_params, 0.1)
    
    # Send goal to action server
    client.send_goal(move_group_goal)

    # Wait for result
    rospy.loginfo("Waiting for motion to complete...")
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    rospy.loginfo("Done!")
    results = client.get_result()
    
    # Handle errors if needed
    rospy.loginfo("Result of pose: " +  moveit_error_dict[results.error_code.val] )
            
