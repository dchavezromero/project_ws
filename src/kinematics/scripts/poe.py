#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from kinematics.common_functions import CommonFunctions as kin


result = Pose()
kin_helper = kin()

def calculate_forward_kin(joints):

    return kin_helper.fkine(kin_helper.S, kin_helper.M, joints, 'space')
                    

def callback():

    joint_angles = kin_helper.get_current_joints_vals()

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
    

    pub = rospy.Publisher('/panda_arm/ee_pose', Pose, queue_size=1)

    r = rospy.Rate(60.0)

    while not rospy.is_shutdown():

        callback()
        # rospy.loginfo(result)

        pub.publish(result)

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass