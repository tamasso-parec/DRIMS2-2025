#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

// ROS Service srv definition

#include "abb_wrapper_msgs/plan_and_execute_pose.h"
#include "abb_wrapper_msgs/open_gripper.h"
#include "abb_wrapper_msgs/close_gripper.h"
#include "abb_wrapper_msgs/plan_and_execute_joint.h"
#include "abb_wrapper_msgs/plan_and_execute_slerp.h"

// Declare ROS clients
ros::ServiceClient plan_and_execute_pose_client;
ros::ServiceClient plan_and_execute_joint_client;
ros::ServiceClient plan_and_execute_slerp_client;
ros::ServiceClient close_gripper_client;
ros::ServiceClient open_gripper_client;

geometry_msgs::Pose grasp_dice_pose;
std_msgs::Int16 dice_value;
int desired_dice_value;
std::string robot;
//
ros::Subscriber dice_value_sub;
ros::Subscriber dice_grasp_sub;

//
bool call_plan_and_execute_pose(geometry_msgs::Pose goal_pose, bool is_relative);
bool call_plan_and_execute_slerp(geometry_msgs::Pose goal_pose, bool is_relative);
bool call_plan_and_execute_joint(std::vector<double> joint_goal);
bool call_open_gripper(bool flag);
bool call_close_gripper(bool flag);

//
void dicevalueCallback(const std_msgs::Int16::ConstPtr &msg)
{
   dice_value.data = msg->data;
   // ROS_INFO("The dice_value is: %d", dice_value.data);
};

void dicePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   grasp_dice_pose.position.x = msg->pose.position.x;
   grasp_dice_pose.position.y = msg->pose.position.y;
   grasp_dice_pose.position.z = msg->pose.position.z;

   grasp_dice_pose.orientation.x = msg->pose.orientation.x;
   grasp_dice_pose.orientation.y = msg->pose.orientation.y;
   grasp_dice_pose.orientation.z = msg->pose.orientation.z;
   grasp_dice_pose.orientation.w = msg->pose.orientation.w;
};

// Function to apply rotation offset
geometry_msgs::Pose applyRotationOffset(const geometry_msgs::Pose &pose, double angle_degrees, const Eigen::Vector3d &axis)
{
   /**
    * @brief This function returns a geometry_msgs::Pose by applying a rotation along (X,Y, or Z axis in degree)
    * @param pose [in] The input geometry_msgs::Pose.
    * @param angle_degrees [in] The input angle degree.
    * @param axis [in] The axis w.r.t to apply the rotation.
    */
   double angle_radians = angle_degrees * M_PI / 180.0;
   Eigen::AngleAxisd rotation(angle_radians, axis); //   axis = Eigen::Vector3d::UnitX() or Eigen::Vector3d::UnitY() or Eigen::Vector3d::UnitZ()
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.rotate(rotation);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiplied the input pose for the RotationOffset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

geometry_msgs::Pose applyDisplacementOffset(const geometry_msgs::Pose &pose, const Eigen::Vector3d &displacement)
{
   /**
    * @brief This function returns a geometry_msgs::Pose by applying a displacement along (X,Y, or Z axis in meters)
    * @param pose [in] The input geometry_msgs::Pose.
    * @param displacement [in] The axis w.r.t to apply the displacement
    */
   // Create an Eigen Affine3d transformation for the displacement
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.translate(displacement);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiply the input pose for the displacement offset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER
**********************************************/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_example");

   ros::NodeHandle nh_;

   // Parse the desired dice value from YAML
   if (!ros::param::get("/dice/dice_top_desired_value", desired_dice_value))
   {
      ROS_WARN("The param 'dice_top_desired_value' not found in param server! ");
   }

   ROS_INFO("The desired dice value is: %d", desired_dice_value);

   // Parse the robot you want to use (gofa or yumi)
   if (nh_.getParam("/control_server_node/robot", robot))
   {
      ROS_INFO("The arm you want to use is: %s", robot.c_str());
   }
   else
   {
      ROS_ERROR("Failed to get '/control_server_node/robot' parameter.");
   }

   // Create subscribers for dice_value and grasp_dice_pose
   dice_value_sub = nh_.subscribe("/dice_value", 1, dicevalueCallback);
   dice_grasp_sub = nh_.subscribe("/dice_pose", 1, dicePoseCallback);

   // Wait for messages on the CallBacks
   ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/dice_pose", ros::Duration(5.0));
   ros::topic::waitForMessage<std_msgs::Int16>("/dice_value", ros::Duration(5.0));

   // Define ROS clients for calling the 5 ROS Services: PlanAndExcutePose, PlanAndExecuteJoint, PlanAndExecuteSlerp, Open and Close Gripper
   if (!ros::service::waitForService("/plan_and_execute_pose", ros::Duration(2.0)))
      return 0;
   plan_and_execute_pose_client = nh_.serviceClient<abb_wrapper_msgs::plan_and_execute_pose>("/plan_and_execute_pose");

   if (!ros::service::waitForService("/plan_and_execute_joint", ros::Duration(2.0)))
      return 0;
   plan_and_execute_joint_client = nh_.serviceClient<abb_wrapper_msgs::plan_and_execute_joint>("/plan_and_execute_joint");

   if (!ros::service::waitForService("/plan_and_execute_slerp", ros::Duration(2.0)))
      return 0;
   plan_and_execute_slerp_client = nh_.serviceClient<abb_wrapper_msgs::plan_and_execute_slerp>("/plan_and_execute_slerp");

   if (!ros::service::waitForService("/open_gripper", ros::Duration(2.0)))
      return 0;
   open_gripper_client = nh_.serviceClient<abb_wrapper_msgs::open_gripper>("/open_gripper");

   if (!ros::service::waitForService("/close_gripper", ros::Duration(2.0)))
      return 0;
   close_gripper_client = nh_.serviceClient<abb_wrapper_msgs::close_gripper>("/close_gripper");

   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
   ros::AsyncSpinner spinner(2);
   spinner.start();

   ROS_INFO("At the beginning of the task");

   /* GRASP DICE TASK*/

   // Home Joint for Gofa (6 joint values)
   std::vector<double> home_joints{-2.24, -0.17, 0.79, -0.11, 1.03, -0.48};

   // Home Joint for YuMi (7 joint values)
   if (robot == "yumi")
   {
      home_joints.push_back(0.0);
   }

   // Plan towards a Home Joint Goal
   bool success = call_plan_and_execute_joint(home_joints);

   //
   ros::spinOnce();

   // Add the displacement along z-axis of the grasp_dice_pose: 6 cm above the grasp_dice_pose
   geometry_msgs::Pose pre_grasp_pose = applyDisplacementOffset(grasp_dice_pose, Eigen::Vector3d(0.0, 0.0, -0.06));

   // open Gripper
   success = call_open_gripper(true);
   // Approach to pre_grasp_pose
   success = call_plan_and_execute_pose(pre_grasp_pose, false);
   //

   // Go into Grasp Pose
   success = call_plan_and_execute_slerp(grasp_dice_pose, false);

   // Close the gripper
   success = call_close_gripper(true);

   // Relative offset of 6 cm in order to plan from the current state of the robot
   geometry_msgs::Pose empty_pose;
   geometry_msgs::Pose displacement_pose = applyDisplacementOffset(empty_pose, Eigen::Vector3d(0.0, 0.0, -0.06));

   // Set true to plan w.r.t a relative offset (the boolean flag is set to true)
   success = call_plan_and_execute_slerp(displacement_pose, true);

   // Open Gripper
   success = call_open_gripper(true);

   // Rotate along X-axis of grasp link frame
   geometry_msgs::Pose rotation_pose = applyRotationOffset(empty_pose, -18.0, Eigen::Vector3d::UnitX());

   //
   success = call_plan_and_execute_slerp(rotation_pose, true);

   //
   ros::waitForShutdown();
   spinner.stop();
   return 0;
}

bool call_plan_and_execute_pose(geometry_msgs::Pose goal_pose, bool is_relative)
{ /**
   * @brief This functions implements a trajectory planning from the current state of the robot or from the last point of the previous past trajectory toward a specified target Cartesian goal.
   *
   *
   * @param goal_pose [in] The Cartesian target goal you want to plan toward.
   * @param is_goal_relative [in] Set True if you want to plan towards a relative goal_pose (previous input)that is relative.
   *                              Example: You want to move the robot w.r.t the current pose of the robot.
   *                              Set False if you want to plan towards an absolute goal_pose (previous input) w.r.t the fixed frame of the robot.
   *                              Example: You want to move the robot from the current_pose toward a absolute global pose.
   */

   abb_wrapper_msgs::plan_and_execute_pose plan_and_execute_pose_srv;
   plan_and_execute_pose_srv.request.goal_pose = goal_pose;
   plan_and_execute_pose_srv.request.is_relative = is_relative;

   if (!plan_and_execute_pose_client.call(plan_and_execute_pose_srv))
   {
      ROS_ERROR("Failed to call the plan_and_execute_pose ROS Service");
      return true;
   }
   else
   {
      ROS_INFO("Call the plan_and_execute_pose ROS Service");
      return false;
   }
}

bool call_plan_and_execute_slerp(geometry_msgs::Pose goal_pose, bool is_relative)
{
   /**
    * @brief This functions implements a trajectory planning from the current state of the robot or from the last point of the previous past trajectory toward a specified target Cartesian goal by using SLERP interpolation.
    *
    *
    * @param goal_pose [in] The Cartesian target goal you want to plan toward.
    * @param is_goal_relative [in] Set True if you want to plan towards a relative goal_pose (previous input)that is relative.
    *                              Example: You want to move the robot w.r.t the current pose of the robot.
    *                              Set False if you want to plan towards an absolute goal_pose (previous input) w.r.t the fixed frame of the robot.
    *                              Example: You want to move the robot from the current_pose toward a absolute global pose.
    */
   abb_wrapper_msgs::plan_and_execute_slerp plan_and_execute_slerp_srv;
   plan_and_execute_slerp_srv.request.goal_pose = goal_pose;
   plan_and_execute_slerp_srv.request.is_relative = is_relative;

   if (!plan_and_execute_slerp_client.call(plan_and_execute_slerp_srv))
   {
      ROS_ERROR("Failed to call the plan_and_execute_slerp ROS Service");
      return true;
   }
   else
   {
      ROS_INFO("Call the plan_and_execute_slerp ROS Service");
      return false;
   }
}

bool call_plan_and_execute_joint(std::vector<double> joint_goal)
{
   /**
    * @brief This functions implements a trajectory planning from the current state of the robot or from the last point of the previous past trajectory toward a joint position goal.
    *
    * @param joint_goal [in] The joint position goal you want to plan toward.
    */
   abb_wrapper_msgs::plan_and_execute_joint plan_and_execute_joint_srv;
   plan_and_execute_joint_srv.request.joint_goal = joint_goal;

   if (!plan_and_execute_joint_client.call(plan_and_execute_joint_srv))
   {
      ROS_ERROR("Failed to call the plan_and_execute_joint ROS Service");
      return false;
   }
   else
   {
      ROS_INFO("Call the plan_and_execute_joint ROS Service");
      return true;
   }
}

bool call_open_gripper(bool in_flag)
{
   /**
    * @brief This functions implements the opening of the gripper
    *
    * @param in_flag [in] Set to true.
    */
   abb_wrapper_msgs::open_gripper open_gripper_srv;
   open_gripper_srv.request.in_flag = in_flag;

   if (!open_gripper_client.call(open_gripper_srv))
   {
      ROS_ERROR("Failed to call the open_gripper ROS Service");
      return false;
   }
   else
   {
      ROS_INFO("Call the open_gripper ROS Service");
      return true;
   }

   return true;
}

bool call_close_gripper(bool in_flag)
{
   /**
    * @brief This functions implements the closing of the gripper
    *
    * @param in_flag [in] Set to true.
    */
   abb_wrapper_msgs::close_gripper close_gripper_srv;
   close_gripper_srv.request.in_flag = in_flag;

   if (!close_gripper_client.call(close_gripper_srv))
   {
      ROS_ERROR("Failed to call the close_gripper ROS Service");
      return false;
   }
   else
   {
      ROS_INFO("Call the close_gripper ROS Service");
      return true;
   }
}
