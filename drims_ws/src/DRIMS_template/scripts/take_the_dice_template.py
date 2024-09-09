#!/usr/bin/env python3

import rospy

from abb_wrapper_msgs.srv import plan_and_execute_pose as PlanAndExecutePose
from abb_wrapper_msgs.srv import plan_and_execute_poseRequest as PlanAndExecutePoseRequest

from abb_wrapper_msgs.srv import plan_and_execute_joint as PlanAndExecuteJoint
from abb_wrapper_msgs.srv import plan_and_execute_jointRequest as PlanAndExecuteJointRequest

from abb_wrapper_msgs.srv import plan_and_execute_slerp as PlanAndExecuteSlerp
from abb_wrapper_msgs.srv import plan_and_execute_slerpRequest as PlanAndExecuteSlerpRequest

from abb_wrapper_msgs.srv import open_gripper as OpenGripper
from abb_wrapper_msgs.srv import open_gripperRequest as OpenGripperRequest

from abb_wrapper_msgs.srv import close_gripper as CloseGripper
from abb_wrapper_msgs.srv import close_gripperRequest as CloseGripperRequest

from geometry_msgs.msg import Pose, PoseStamped

DICE_POSE_TOPIC  = '/dice_pose'
PLAN_AND_EXECUTE_POSE_SERVICE_NAME  = '/plan_and_execute_pose'
PLAN_AND_EXECUTE_JOINT_SERVICE_NAME = '/plan_and_execute_joint'
PLAN_AND_EXECUTE_SLERP_SERVICE_NAME = '/plan_and_execute_slerp'
OPEN_GRIPPER_SERVICE_NAME           = '/open_gripper'
CLOSE_GRIPPER_SERVICE_NAME          = '/close_gripper'

def plan_and_execute_joint(joint_goal):
    rospy.loginfo(f'PLAN AND EXECUTE TO A JOINT CONFIGURATION')
    rospy.wait_for_service(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME)
    plan_and_execute_joint_client = rospy.ServiceProxy(PLAN_AND_EXECUTE_JOINT_SERVICE_NAME, PlanAndExecuteJoint)

    srv_request = PlanAndExecuteJointRequest()
    srv_request.joint_goal = joint_goal

    try:
        plan_and_execute_joint_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
    
    return

def plan_and_execute_pose(goal_pose, is_relative):
    rospy.loginfo(f'PLAN AND EXECUTE TO A CARTESIAN POSE WITH JOINT TRAJECTORY. IS A RELATIVE MOVE: {is_relative}')
    rospy.loginfo(f'Waiting {PLAN_AND_EXECUTE_POSE_SERVICE_NAME}')

    # Create a client of the service
    plan_and_execute_pose_client = rospy.ServiceProxy(PLAN_AND_EXECUTE_POSE_SERVICE_NAME, PlanAndExecutePose)

    # Create request
    srv_request = PlanAndExecutePoseRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative

    try:
        plan_and_execute_pose_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
    
    return


def plan_and_execute_slerp(goal_pose, is_relative):
    rospy.loginfo(f'PLAN AND EXECUTE TO A CARTESIAN POSE WITH CARTESIAN TRAJECTORY. IS A RELATIVE MOVE: {is_relative}')
    rospy.wait_for_service(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME)
    plan_and_execute_slerp_client = rospy.ServiceProxy(PLAN_AND_EXECUTE_SLERP_SERVICE_NAME, PlanAndExecuteSlerp)

    srv_request = PlanAndExecuteSlerpRequest()
    srv_request.goal_pose = goal_pose
    srv_request.is_relative = is_relative
    
    try:
        plan_and_execute_slerp_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')
    
    return


def open_gripper():
    rospy.loginfo(f'OPEN GRIPPER')
    rospy.wait_for_service(OPEN_GRIPPER_SERVICE_NAME)
    open_gripper_client = rospy.ServiceProxy(OPEN_GRIPPER_SERVICE_NAME, OpenGripper)

    srv_request = OpenGripperRequest()
    srv_request.in_flag = True

    try:
        response = open_gripper_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

def close_gripper():
    rospy.loginfo(f'CLOSE GRIPPER')
    rospy.wait_for_service(CLOSE_GRIPPER_SERVICE_NAME)
    close_gripper_client = rospy.ServiceProxy(CLOSE_GRIPPER_SERVICE_NAME, CloseGripper)

    srv_request = CloseGripperRequest()
    srv_request.in_flag = True

    try:
        response = close_gripper_client(srv_request)
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

def wait_for_dice_pose():
    dice_pose = rospy.wait_for_message(DICE_POSE_TOPIC, PoseStamped, timeout=None)
    rospy.loginfo(f'Pose received: {dice_pose}')
    return dice_pose.pose

def main():
    rospy.init_node('take_the_dice_example')
    
    # ******************************
    # **** Write here your code ****
    # ******************************




    # ******************************


if __name__ == '__main__':
    main()
