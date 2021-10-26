#!/usr/bin/env python

from __future__ import print_function

import rospy
import math
from kinematics.srv import MoveJoint,MoveJointResponse
from kinematics.msg import joint_angles

pub_setPoint = rospy.Publisher("/panda/set_points", joint_angles, queue_size=1)
set_points = joint_angles()

joint_1_limits = [-2.8973, 2.8973]
joint_2_limits = [-1.7628, 1.7628]
joint_3_limits = [-2.8973, 2.8973]
joint_4_limits = [-3.0718, 3.0718]
joint_4_limits = [-3.0718, 0.0698]
joint_5_limits = [-2.8973, 2.8973]
joint_6_limits = [-0.0175, 3.7525]
joint_7_limits = [-2.8973, 2.8973]

def callback(req):
    response = False

    theta1 = req.joint_set_points.theta1
    theta2 = req.joint_set_points.theta2
    theta2 = req.joint_set_points.theta2
    theta3 = req.joint_set_points.theta3
    theta4 = req.joint_set_points.theta4
    theta5 = req.joint_set_points.theta5
    theta6 = req.joint_set_points.theta6
    theta7 = req.joint_set_points.theta7

    # Maybe change to joint limits to be referenced from xacro file
    if ((theta1 <= joint_1_limits[1] and theta1 >= joint_1_limits[0]) 
    and (theta2 <= joint_2_limits[1] and theta2 >= joint_2_limits[0])
    and (theta3 <= joint_3_limits[1] and theta3 >= joint_3_limits[0])
    and (theta4 <= joint_4_limits[1] and theta4 >= joint_4_limits[0])
    and (theta5 <= joint_5_limits[1] and theta5 >= joint_5_limits[0])
    and (theta6 <= joint_6_limits[1] and theta6 >= joint_6_limits[0])
    and (theta7 <= joint_7_limits[1] and theta7 >= joint_7_limits[0])):

        response = True

        set_points.theta1 = theta1
        set_points.theta2 = theta2
        set_points.theta3 = theta3
        set_points.theta4 = theta4
        set_points.theta5 = theta5
        set_points.theta6 = theta6
        set_points.theta7 = theta7

        pub_setPoint.publish(set_points)

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    rospy.spin()

if __name__ == "__main__":
    position_controller_server()