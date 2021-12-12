#!/usr/bin/env python

from kinematics.srv import InvKin,InvKinResponse
from kinematics.msg import joint_angles
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix
from kinematics.common_functions import CommonFunctions as kin
import modern_robotics as mr
import numpy as np
import rospy
import sympy as sym

result = joint_angles()
current_pose = Pose()
kin_helper = kin()

def update_current_pose(msg):
    global current_pose
    current_pose = msg

def calc_current_T():
    global current_pose
    temp_matrix = quaternion_matrix([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
    
    R = sym.Matrix(temp_matrix[0:3, 0:3])

    p = sym.Matrix([current_pose.position.x, current_pose.position.y, current_pose.position.z])

    top = sym.Matrix(sym.BlockMatrix([[R, p]]))
    bot = sym.Matrix([[0, 0, 0, 1]])

    return sym.Matrix(sym.BlockMatrix([[top], [bot]]))


def calc_inv_kin(req):

    valid_pose = True

    current_position = sym.Matrix([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    target_position = sym.Matrix([req.target_pose.x, req.target_pose.y, req.target_pose.z])

    currentQ = kin_helper.get_current_joints_vals()
    timeout_counter=0

    T = calc_current_T()


    while((kin_helper.norm(target_position - current_position) > 1e-3) or not kin_helper.check_limits(currentQ)):

        J_a = kin_helper.jacoba(sym.transpose(kin_helper.S_body), currentQ, T)

        temp_matrix = J_a * sym.transpose(J_a) 
        deltaQ = sym.transpose(J_a) * temp_matrix.inv() * (target_position - current_position)

        currentQ = currentQ + deltaQ
        

        T = sym.Matrix(mr.FKinSpace(np.array(kin_helper.M).astype(np.float64), 
                                            np.array(kin_helper.S_space).astype(np.float64), 
                                            np.array(currentQ).astype(np.float64)))
        current_position = T[0:3, 3]
        
        timeout_counter+=1

        if timeout_counter == 5:
            valid_pose = False
            error_message = "Inverse kinematics timed-out, unsolvable position, default to start configuration!"
            rospy.logerr(error_message)
            break

    if valid_pose == True:
        result.theta1 = currentQ[0]
        result.theta2 = currentQ[1]
        result.theta3 = currentQ[2]
        result.theta4 = currentQ[3]
        result.theta5 = currentQ[4]
        result.theta6 = currentQ[5]
        result.theta7 = currentQ[6]
    else:   
        result.theta1 = 0.0 
        result.theta2 = -0.6 
        result.theta3 = 0.0 
        result.theta4 = -1.5 
        result.theta5 = 0.0
        result.theta6 = 2.4 
        result.theta7 = 0.0 

    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    rospy.Subscriber('/panda_arm/ee_pose', Pose, update_current_pose)

    init_msg = "Ready to calculate inverse kinematics!"
    rospy.loginfo(init_msg)

    rospy.spin()
    

if __name__ == "__main__":
    calc_inv_kin_server()