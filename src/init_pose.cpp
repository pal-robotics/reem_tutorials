#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace reem_tutorials
{

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

} // reem_tutorials

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
  reem_tutorials::MoveGroup right_arm_torso("right_arm_torso_controller");
  reem_tutorials::MoveGroup left_arm("left_arm_controller");
  reem_tutorials::MoveGroup head("head_traj_controller");

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