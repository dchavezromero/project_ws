#!/usr/bin/env python
import rospy
import sympy as sym
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose
from kinematics.common_functions import CommonFunctions as kin


result = Pose()

def calculate_forward_kin(joints):
    # Link length of values (meters)
    L0 = 0.088
    L1 = 0.333
    L2 = 0.316
    L3 = 0.384
    L4 = 0.107
    L5 = 0.0825

    S = sym.Matrix([[0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 0, -1, 0, -1, 0],
                    [1, 0, 1, 0, 1, 0, -1],
                    [0, -L1, 0, L1+L2, 0, L1+L2+L3, 0],
                    [0, 0, 0, 0, 0, 0, L0],
                    [0, 0, 0, -L5, 0, 0, 0]])

    R = sym.Matrix([[1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]])

    p = sym.Matrix([L0, 0, L1+L2+L3-L4])

    top = sym.Matrix(sym.BlockMatrix([[R, p]]))
    bot = sym.Matrix([[0, 0, 0, 1]])

    M = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

    return kin_helper.fkine(S, M, joints, 'space')
                    

def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def callback():
    panda_joint1 = get_position('panda_joint1')
    panda_joint2 = get_position('panda_joint2')
    panda_joint3 = get_position('panda_joint3')
    panda_joint4 = get_position('panda_joint4')
    panda_joint5 = get_position('panda_joint5')
    panda_joint6 = get_position('panda_joint6')
    panda_joint7 = get_position('panda_joint7')

    joint_angles = panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7

    trans_matrix = calculate_forward_kin(joint_angles)

    # rospy.loginfo(trans_matrix)

    ee_pos = trans_matrix[0,3], trans_matrix[1,3], trans_matrix[2,3]
    rot_matrix = trans_matrix[0,0], trans_matrix[0,1], trans_matrix[0,2], \
                 trans_matrix[1,0], trans_matrix[1,1], trans_matrix[1,2], \
                 trans_matrix[2,0], trans_matrix[2,1], trans_matrix[2,2]

    quat = kin_helper.get_quaternions(rot_matrix)

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
    pub = rospy.Publisher("/panda_arm/ee_pose", Pose, queue_size=1)

    r = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        callback()
        # rospy.loginfo(result)

        pub.publish(result)

        r.sleep()

if __name__ == "__main__":
    try:
        kin_helper = kin()
        main()
    except rospy.ROSInterruptException:
        pass