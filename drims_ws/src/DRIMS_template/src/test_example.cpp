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
   std::vector<double>home_joints;
   // Home Joint for GoFa (6 joint values)
   if (robot == "gofa")
   {
      home_joints.assign({-1.98, 0.0, 0.51, -0.07, 0.94, 0.0});
   }

   // Home Joint for YuMi(7 joint values)
   if (robot == "yumi")
   {  
      home_joints.assign({0.5778451033162864, -0.4259838682137797, 2.765560699257221, 0.40098920256651815, 0.2795610410838103, 0.8403600825003401, -1.0629680588027473});
   }
   
   /* Task of MANIPULATION DICE*/

   // 1) Plan and go towards a HOME JOINTS goal
   bool success = call_plan_and_execute_joint(home_joints);

   // 2) OPEN the gripper (Schunk for Gofa and Smart Gripper for YuMi)
   success = call_open_gripper(true);

   // Plan towards a PRE GRASP POSE (6 cm above the GRASP POSE of the dice)
   ros::spinOnce();
   geometry_msgs::Pose pre_grasp_pose = grasp_dice_pose;
   pre_grasp_pose.position.z += 0.06;

   // 3) Plan and go towards PRE GRASP POSE
   success = call_plan_and_execute_pose(pre_grasp_pose, false);
   
   // 4) Plan and go towards a GRASP POSE
   success = call_plan_and_execute_slerp(grasp_dice_pose, false);

   // 5) CLOSE the gripper
   success = call_close_gripper(true);

   // 6)  Plan and go towards PRE GRASP POSE
   success = call_plan_and_execute_slerp(pre_grasp_pose, false);

   // 7) OPEN the gripper
   success = call_open_gripper(true);
   
   /**/

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
