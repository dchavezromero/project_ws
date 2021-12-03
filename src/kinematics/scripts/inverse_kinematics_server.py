#!/usr/bin/env python

from kinematics.srv import InvKin,InvKinResponse
from kinematics.msg import joint_angles
from geometry_msgs.msg import Pose, Point
import rospy
import sympy as sym
from kinematics.common_functions import CommonFunctions as kin

result = joint_angles()
current_pose = Point()
kin_helper = kin()

def update_current_pose(msg):
    current_pose.x = msg.position.x
    current_pose.y = msg.position.y
    current_pose.z = msg.position.z

    # rospy.loginfo(current_pose)

def calc_inv_kin(req):

    # print(req.target_pose.x)

    current_position = sym.Matrix([current_pose.x, current_pose.y, current_pose.z])
    target_position = sym.Matrix([req.target_pose.x, req.target_pose.y, req.target_pose.z])

    currentQ = sym.transpose(kin_helper.get_current_joints_vals())

    while((kin_helper.norm(target_position - current_position) > 1e-3) or not kin_helper.check_limits(currentQ)):
        print(kin_helper.norm(target_position - current_position))

        J_a = kin_helper.jacoba(kin_helper.S, kin_helper.M, currentQ)
        # rospy.loginfo(current_position)
        # print(J_a)

        lambda_val = 1

        temp_matrix = J_a * sym.transpose(J_a) * lambda_val**2 * sym.eye(3)

        deltaQ = sym.transpose(J_a) * temp_matrix.pinv() * (target_position - current_position)

        
        # print(currentQ)
        # print(sym.transpose(deltaQ))
        currentQ = currentQ + sym.transpose(deltaQ)

        # print("here")

        T = kin_helper.fkine(kin_helper.S, kin_helper.M, currentQ, 'space')
        current_position = T[0:3, 3]

        # print(kin_helper.norm(target_position - current_position))

    result.theta1 = currentQ[0]
    result.theta2 = currentQ[1]
    result.theta3 = currentQ[2]
    result.theta4 = currentQ[3]
    result.theta5 = currentQ[4]
    result.theta6 = currentQ[5]
    result.theta7 = currentQ[6]

    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    rospy.Subscriber('/panda_arm/ee_pose', Pose, update_current_pose)

    print("Ready to calculate inverse kinematics.")
    rospy.spin()
    

if __name__ == "__main__":
    calc_inv_kin_server()