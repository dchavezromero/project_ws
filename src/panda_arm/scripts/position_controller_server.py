#!/usr/bin/env python

from __future__ import print_function

import rospy
from kinematics.srv import MoveJoint,MoveJointResponse
from kinematics.msg import joint_angles
from std_msgs.msg import Float64
from kinematics.common_functions import CommonFunctions as kin

kin_helper = kin()
ros_rate = 50.0
set_points = joint_angles()

pub_set_points_topic = rospy.Publisher("/panda_arm/set_points", joint_angles, queue_size=1)

update_joint1 = rospy.Publisher("/panda_arm/panda_joint1_position_controller/command", Float64, queue_size=1)
update_joint2 = rospy.Publisher("/panda_arm/panda_joint2_position_controller/command", Float64, queue_size=1)
update_joint3 = rospy.Publisher("/panda_arm/panda_joint3_position_controller/command", Float64, queue_size=1)
update_joint4 = rospy.Publisher("/panda_arm/panda_joint4_position_controller/command", Float64, queue_size=1)
update_joint5 = rospy.Publisher("/panda_arm/panda_joint5_position_controller/command", Float64, queue_size=1)
update_joint6 = rospy.Publisher("/panda_arm/panda_joint6_position_controller/command", Float64, queue_size=1)
update_joint7 = rospy.Publisher("/panda_arm/panda_joint7_position_controller/command", Float64, queue_size=1)
update_finger1 = rospy.Publisher("/panda_arm/panda_finger_joint1_controller/command", Float64, queue_size=1)
update_finger2 = rospy.Publisher("/panda_arm/panda_finger_joint2_controller/command", Float64, queue_size=1)


def update_joint_positions():
    update_joint1.publish(set_points.theta1)
    update_joint2.publish(set_points.theta2)
    update_joint3.publish(set_points.theta3)
    update_joint4.publish(set_points.theta4)
    update_joint5.publish(set_points.theta5)
    update_joint6.publish(set_points.theta6)
    update_joint7.publish(set_points.theta7)
    update_finger1.publish(set_points.finger1)
    update_finger2.publish(set_points.finger2)

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
        # print("here")
        pub_set_points_topic.publish(set_points)

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    # rospy.spin()

    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        pub_set_points_topic.publish(set_points)


if __name__ == "__main__":
    position_controller_server()