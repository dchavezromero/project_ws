#!/usr/bin/env python

from kinematics.srv import InvKin,InvKinResponse
from kinematics.msg import joint_angles
from geometry_msgs.msg import Pose, Point
import rospy
import time
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
    start_time = time.time()
    counter=0


    temp_total_J_a_time = 0
    temp_total_delQ_time = 0

    while((kin_helper.norm(target_position - current_position) > 1e-3) or not kin_helper.check_limits(currentQ)):
        # print(kin_helper.norm(target_position - current_position))
    
        start_time_J_a = time.time()
        J_a = kin_helper.jacoba(kin_helper.S, kin_helper.M, currentQ)
        end_time_J_a = time.time()

        temp_total_J_a_time = end_time_J_a - start_time_J_a

       # print(J_a)
        # rospy.loginfo(current_position)
        # print(J_a)

        # lambda_val = 1

        temp_matrix = J_a * sym.transpose(J_a) # * lambda_val**2 * sym.eye(3)


        start_time_delQ = time.time()
        deltaQ = sym.transpose(J_a) * temp_matrix.inv() * (target_position - current_position)
        end_time_delQ = time.time()

        temp_total_delQ_time = end_time_delQ - start_time_delQ


        #deltaQ = J_a.pinv() * (target_position - current_position)

        #print("calculated inv at iter: ", counter)
        
        # print(currentQ)
        # print(sym.transpose(deltaQ))
        currentQ = currentQ + sym.transpose(deltaQ)

        # print("here")

        T = kin_helper.fkine(kin_helper.S, kin_helper.M, currentQ, 'space')
        current_position = T[0:3, 3]
        counter+=1

        temp_total_J_a_time += temp_total_J_a_time
        temp_total_delQ_time += temp_total_delQ_time

        # print(kin_helper.norm(target_position - current_position))

    result.theta1 = currentQ[0]
    result.theta2 = currentQ[1]
    result.theta3 = currentQ[2]
    result.theta4 = currentQ[3]
    result.theta5 = currentQ[4]
    result.theta6 = currentQ[5]
    result.theta7 = currentQ[6]

    end_time = time.time()
    print("iters in loop: ", counter)
    time_elapsed = end_time - start_time
    print("Time to calc inv kin: ", time_elapsed, " seconds")

    print("total time calculating analytical jacobians: ", temp_total_J_a_time)
    print("total time calcuting deltaQ: ", temp_total_delQ_time)

    return InvKinResponse(result)

def calc_inv_kin_server():
    rospy.init_node('inverse_kinematics_server')
    s = rospy.Service('inverse_kinematics', InvKin, calc_inv_kin)
    rospy.Subscriber('/panda_arm/ee_pose', Pose, update_current_pose)

    print("Ready to calculate inverse kinematics.")
    rospy.spin()
    

if __name__ == "__main__":
    calc_inv_kin_server()