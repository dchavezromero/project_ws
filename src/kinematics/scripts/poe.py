#!/usr/bin/env python
import rospy
import numpy as np
import sympy as sym
import modern_robotics as mr
from geometry_msgs.msg import Pose
from kinematics.common_functions import CommonFunctions as kin

from tf.transformations import quaternion_from_matrix

# import time


result = Pose()
kin_helper = kin()

def callback():

    joint_angles = kin_helper.get_current_joints_vals()

    # start_time = time.time()
    # trans_matrix = kin_helper.fkine(kin_helper.S, kin_helper.M, joint_angles, 'space')

    trans_matrix = sym.Matrix(mr.FKinSpace(np.array(kin_helper.M).astype(np.float64), 
                                            np.array(kin_helper.S_space).astype(np.float64), 
                                            np.array(joint_angles).astype(np.float64)))
    # end_time = time.time()

    # elapsed_time = end_time - start_time

    # print("Time to calc forw kin: ", 1/elapsed_time, " Hz")
    # rospy.loginfo(trans_matrix)
    ee_pos = trans_matrix[0:3,3]
    # rot_matrix = trans_matrix[0:3,0:3]

    # print(np.array(rot_matrix).astype(np.float64))

    quat = quaternion_from_matrix(np.array(trans_matrix).astype(np.float64))

    # print(quat)

    result.position.x = ee_pos[0]
    result.position.y = ee_pos[1]
    result.position.z = ee_pos[2]
    result.orientation.x = quat[0]
    result.orientation.y = quat[1]
    result.orientation.z = quat[2]
    result.orientation.w = quat[3]

def main():
    # M = sym.Matrix([[0, 1, 0, 0],[0, 0, -1, 0.3], [-1, 0, 0, 0.15], [0, 0, 0, 1]])

    # S = sym.Matrix([

    #      [0,    1,   1],
    #      [0,         0,         0],
    # [1,         0,         0],
    #      [0,        0,         0],
    #      [0,    0.3,    0.3],
    #      [0,         0,   -0.3]])

    # q = sym.transpose(sym.Matrix([0.4, 0.3, 0.2]))

    # J_s = kin_helper.jacob0(S,q)
    # print(J_s)
    # T = kin_helper.fkine(S,M,q,'space')
    # print(T)
    # print(kin_helper.twist_space_2_body(J_s, T))

    # print(kin_helper.jacoba(S, M, q))

    rospy.init_node("forward_kinematics")
    

    pub = rospy.Publisher('/panda_arm/ee_pose', Pose, queue_size=1)

    r = rospy.Rate(400.0)

    while not rospy.is_shutdown():

        callback()
        # rospy.loginfo("test")

        pub.publish(result)

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass