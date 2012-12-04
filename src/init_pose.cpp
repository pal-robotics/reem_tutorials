/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Adolfo Rodriguez Tsouroukdissian. */
// NOTE: The contents of this file are based on the JointTrajectoryAction tutorial available here:
// http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class MoveGroup
{
public:
  MoveGroup(const std::string& controller_name);
  bool sendGoal(const std::vector<double>& pose, const ros::Duration& duration);
  const std::vector<std::string>& getJointNames() const {return joint_names_;}
  actionlib::SimpleClientGoalState getState() {return client_.getState();}

private:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;

  std::string controller_name_;          ///< Controller name.
  std::vector<std::string> joint_names_; ///< Names of controller joints.
  ActionClient client_;                  ///< Action client used to trigger motions.

  bool configure();
};

MoveGroup::MoveGroup(const std::string& controller_name)
  : controller_name_(controller_name),
    client_(controller_name_ + "/follow_joint_trajectory")
{
  if (!configure())
  {
    throw;
  }
}

bool MoveGroup::configure()
{
  // Wait for the action client to be connected to the server
  if (!client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR_STREAM("Timed-out waiting for the \'" << controller_name_ << "\' FollowJointTrajectory action server.");
    return false;
  }

  // Get list of joints used by the controller
  joint_names_.clear();
  using namespace XmlRpc;
  XmlRpcValue joint_names;
  ros::NodeHandle nh(controller_name_);
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                nh.getNamespace().c_str());
      return false;
    }
    joint_names_.push_back(static_cast<std::string>(name_value));
  }
  return true;
}

bool MoveGroup::sendGoal(const std::vector<double>& pose, const ros::Duration& duration)
{
  // Goal pose for right_arm_torso group
  if (pose.size() != joint_names_.size())
  {
    ROS_ERROR_STREAM("Pose size mismatch. Expected: " << joint_names_.size() << ", got: " << pose.size() << ".");
    return false;
  }
  ActionGoal goal;
  goal.trajectory.joint_names = joint_names_;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions = pose;                            // Reach these joint positions...
  goal.trajectory.points[0].velocities.resize(joint_names_.size(), 0.0); // ...with zero-velocity
  goal.trajectory.points[0].time_from_start = duration;                  // ...in this time

  client_.sendGoal(goal);
  return true;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "init_posture");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Instances that will move each joint group to the desired pose
  MoveGroup right_arm_torso("right_arm_torso_controller");
  MoveGroup left_arm("left_arm_controller");
  MoveGroup head("head_traj_controller");

  // Desired poses for each joint group
  std::vector<double> right_arm_torso_pose(right_arm_torso.getJointNames().size(), 0.0);
  right_arm_torso_pose[2] = -0.4;    // arm_right_1_joint NOTE: torso joints are indices 0 and 1
  right_arm_torso_pose[3] =  0.1;    // arm_right_2_joint
  right_arm_torso_pose[4] = -0.1;    // arm_right_3_joint
  right_arm_torso_pose[5] =  0.6109; // arm_right_4_joint

  std::vector<double> left_arm_pose(left_arm.getJointNames().size(), 0.0);
  left_arm_pose[0] = -0.4;    // arm_left_1_joint
  left_arm_pose[1] =  0.1;    // arm_left_2_joint
  left_arm_pose[2] = -0.1;    // arm_left_3_joint
  left_arm_pose[3] =  0.6109; // arm_left_4_joint

  std::vector<double> head_pose(head.getJointNames().size(), 0.0);

  // Send pose commands
  right_arm_torso.sendGoal(right_arm_torso_pose, ros::Duration(2.0));
  left_arm.sendGoal(left_arm_pose, ros::Duration(2.0));
  head.sendGoal(head_pose, ros::Duration(2.0));

  // Wait until pose is reached
  while(!right_arm_torso.getState().isDone() &&
        !left_arm.getState().isDone() &&
        !head.getState().isDone() &&
        ros::ok())
  {
    ROS_INFO("Waiting for motion to complete...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Done!");

  return EXIT_SUCCESS;
}