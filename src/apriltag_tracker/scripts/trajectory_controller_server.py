#!/usr/bin/env python

import rospy
from apriltag_tracker.srv import MoveJoint,MoveJointResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kinematics.msg import joint_angles
from kinematics.common_functions import CommonFunctions as kin

kin_helper = kin()
ros_rate = 50.0
set_points = joint_angles()

pub_set_points_topic = rospy.Publisher("/panda_arm/set_points", joint_angles, queue_size=1)

update_trajectory = rospy.Publisher('/panda_arm/arm_controller/command', JointTrajectory, queue_size=1)

def update_joint_positions():
    arm_trajectory = JointTrajectory()
    goal_positions = [set_points.theta1,
    set_points.theta2,
    set_points.theta3,
    set_points.theta4,
    set_points.theta5,
    set_points.theta6,
    set_points.theta7]

    arm_trajectory.joint_names = ["panda_joint1", 
    "panda_joint2", 
    "panda_joint3", 
    "panda_joint4", 
    "panda_joint5", 
    "panda_joint6", 
    "panda_joint7"]

    arm_trajectory.points.append(JointTrajectoryPoint())
    arm_trajectory.points[0].positions = goal_positions
    arm_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
    arm_trajectory.points[0].accelerations = [0, 0, 0, 0, 0, 0, 0]
    arm_trajectory.points[0].time_from_start = rospy.Duration(0.5)

    update_trajectory.publish(arm_trajectory)

def callback(req):

    theta1 = req.joint_set_points.theta1
    theta2 = req.joint_set_points.theta2
    theta3 = req.joint_set_points.theta3
    theta4 = req.joint_set_points.theta4
    theta5 = req.joint_set_points.theta5
    theta6 = req.joint_set_points.theta6
    theta7 = req.joint_set_points.theta7
    finger1 = req.joint_set_points.finger1
    finger2 = req.joint_set_points.finger2

    joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6, theta7, finger1, finger2]

    response = kin_helper.check_limits(joint_angles)

    if(response):
        set_points.theta1 = theta1
        set_points.theta2 = theta2
        set_points.theta3 = theta3
        set_points.theta4 = theta4
        set_points.theta5 = theta5
        set_points.theta6 = theta6
        set_points.theta7 = theta7
        set_points.finger1 = finger1
        set_points.finger2 = finger2

        update_joint_positions()
        pub_set_points_topic.publish(set_points)

    return MoveJointResponse(response)

def trajectory_controller_server():
    rospy.init_node('trajectory_controller_server')
    s = rospy.Service('trajectory_controller', MoveJoint, callback)
    init_msg = "Ready to drive joints!"
    rospy.loginfo(init_msg)

    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        pub_set_points_topic.publish(set_points)


if __name__ == "__main__":
    trajectory_controller_server()