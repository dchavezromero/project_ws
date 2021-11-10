#!/usr/bin/env python

from __future__ import print_function

import rospy
from kinematics.srv import MoveJoint,MoveJointResponse
from kinematics.msg import joint_angles
from std_msgs.msg import Float64

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

set_points = joint_angles()

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

def check_limits(req):
    response = False

    theta1 = req.joint_set_points.theta1
    
    theta3 = req.joint_set_points.theta3
    theta4 = req.joint_set_points.theta4
    theta5 = req.joint_set_points.theta5
    theta6 = req.joint_set_points.theta6
    theta7 = req.joint_set_points.theta7
    finger1 = req.joint_set_points.finger1
    finger2 = req.joint_set_points.finger2
    theta2 = req.joint_set_points.theta2
    theta3 = req.joint_set_points.theta3
    theta2 = req.joint_set_points.theta2

    # print(type(theta1))
    # print((theta1 <= 2.8973 and theta1 >= -2.8973))

    # Maybe change to joint limits to be referenced from xacro file
    if ((theta1.data <= 2.8973 and theta1.data >= -2.8973) 
    and (theta2.data <= 1.7628 and theta2.data >= -1.7628) 
    and (theta3.data <= 2.8973 and theta3.data >= -2.8973) 
    and (theta4.data <= 0.0698 and theta4.data >= -3.0718) 
    and (theta5.data <= 2.8973 and theta5.data >= -2.8973) 
    and (theta6.data <= 3.7525 and theta6.data >= -0.0175) 
    and (theta7.data <= 2.8973 and theta7.data >= -2.8973)
    and (finger1.data <= 0.04 and finger1.data >= -0.001) 
    and (finger2.data <= 0.04 and finger2.data >= -0.001)):

        response = True

        set_points.theta1 = theta1
        set_points.theta2 = theta2
        set_points.theta3 = theta1
        set_points.theta4 = theta2
        set_points.theta5 = theta1
        set_points.theta6 = theta2
        set_points.theta7 = theta1
        set_points.finger1 = theta2
        set_points.finger2 = theta1

        # pub_setPoint.publish(set_points)

    return response

def callback(req):
    response = check_limits(req)

    if(response):
        update_joint_positions()
        # print("here")
        pub_set_points_topic.publish(set_points)

    return MoveJointResponse(response)

def position_controller_server():
    rospy.init_node('position_controller_server')
    s = rospy.Service('position_controller', MoveJoint, callback)
    print("Ready to drive joints.")
    rospy.spin()

if __name__ == "__main__":
    position_controller_server()