#!/usr/bin/env python

import rospy
import numpy as np
import sympy as sym
import modern_robotics as mr
from geometry_msgs.msg import Pose
from kinematics.common_functions import CommonFunctions as kin
from tf.transformations import quaternion_from_matrix


result = Pose()
kin_helper = kin()

def do_forward_kin():

    joint_angles = kin_helper.get_current_joints_vals()

    trans_matrix = sym.Matrix(mr.FKinSpace(np.array(kin_helper.M).astype(np.float64), 
                                            np.array(kin_helper.S_space).astype(np.float64), 
                                            np.array(joint_angles).astype(np.float64)))


    ee_pos = trans_matrix[0:3,3]
    quat = quaternion_from_matrix(np.array(trans_matrix).astype(np.float64))

    result.position.x = ee_pos[0]
    result.position.y = ee_pos[1]
    result.position.z = ee_pos[2]
    result.orientation.x = quat[0]
    result.orientation.y = quat[1]
    result.orientation.z = quat[2]
    result.orientation.w = quat[3]

def main():

    rospy.init_node("forward_kinematics")
    init_msg = "Ready to perform forward kinematics!"
    rospy.loginfo(init_msg)
    pub = rospy.Publisher('/panda_arm/ee_pose', Pose, queue_size=1)
    r = rospy.Rate(400.0)

    while not rospy.is_shutdown():
        do_forward_kin()
        pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass