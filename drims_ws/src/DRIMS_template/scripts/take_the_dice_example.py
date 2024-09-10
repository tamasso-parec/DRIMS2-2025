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
    
    rospy.loginfo('Going to home...')
    goal_joints = []
    if not rospy.has_param('/control_server_node/robot'):
        rospy.logerr('No robot loaded')
        return
    
    robot_name = rospy.get_param('/control_server_node/robot')
    if robot_name == 'yumi_single_arm' or robot_name == 'yumi':
        goal_joints = [-2.12, -0.1, 0.6, -0.04, 0.16, 1.6, -2.56]
        # goal_joints = [-1.2, 0.18, -0.37, 0.01, 0.05, 1.4, -1.6]
        # goal_joints = [0.178, 0.137, 0.0, 0.0, -0.1, 1.6, -1.28] # joints 1 to 7
    else:
        goal_joints = [0.0, -0.4, 0.6, 0.0, 1.36, -1.30] # joints 1 to 6
    
    plan_and_execute_joint(goal_joints)

    rospy.loginfo('Waiting for the dice pose...')
    dice_pose = wait_for_dice_pose()
    
    rospy.loginfo('Approaching the dice...')
    approach_pose = dice_pose
    approach_pose.position.z = approach_pose.position.z+0.3 #30 centimeters above the dice

    plan_and_execute_pose(approach_pose, False)

    rospy.loginfo('Opening gripper...')
    open_gripper()

    rospy.loginfo('Going down on the dice...')
    down_pose = Pose()
    down_pose.position.z = 0.20
    plan_and_execute_slerp(down_pose, True)

    rospy.loginfo('Closing gripper...')
    close_gripper()

    rospy.loginfo('Going up...')
    up_pose = Pose()
    up_pose.position.z = -0.20 
    plan_and_execute_slerp(up_pose, True)

    rospy.loginfo('Releasing the dice...')
    release_pose = Pose()
    release_pose.position.x = 0.05 
    release_pose.position.y = 0.05 
    release_pose.position.z = 0.20
    plan_and_execute_slerp(release_pose, True)

    rospy.loginfo('Opening gripper...')
    open_gripper()

    rospy.loginfo('Going up...')
    up_pose = Pose()
    up_pose.position.z = -0.20 
    plan_and_execute_slerp(up_pose, True)

if __name__ == '__main__':
    main()
