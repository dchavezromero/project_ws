#!/usr/bin/env python
import rospy
import math
import sympy as sym
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose

result = Pose()

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    quaternions = qw, qx, qy, qz

    # rospy.loginfo(quaternions)

    return quaternions

def get_quaternions(rot_matrix):
    roll = math.atan2(rot_matrix[7], rot_matrix[8])
    pitch = math.atan2(-rot_matrix[6], math.sqrt(pow(rot_matrix[7], 2) + pow(rot_matrix[8], 2)))
    yaw = math.atan2(rot_matrix[3], rot_matrix[0])

    euler_angles = roll, pitch, yaw

    # rospy.loginfo(euler_angles)

    quaternions = euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
    return quaternions

def skew(v):
    if len(v) == 3:
        S = sym.Matrix([[0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]])
    else:
        rospy.logerr("argument must be a 3-vector")

    return S

def axis_angle_2_rot(omega, theta):
    omega_ss = skew(omega)
    R = sym.eye(3) + math.sin(theta) * omega_ss + (1-math.cos(theta)) * omega_ss**2

    return R

def twist_2_ht(S, theta):
    w = S[0:3,:]
    v = S[3:6,:]

    R = axis_angle_2_rot(w,theta)

    s_skew = skew(w)

    col = ((sym.eye(3)*theta) + (1-math.cos(theta))*s_skew + (theta-math.sin(theta))*(s_skew**2))*v

    top = sym.Matrix(sym.BlockMatrix([[R, col]]))
    bot = sym.Matrix([[0, 0, 0, 1]])

    T = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

    return T

def fkine(S, M, q):
    n_joints = len(q)

    T_n = sym.eye(4)

    for n in range(n_joints):
        T_n = T_n*twist_2_ht(S[:,n], q[n])

    return T_n*M

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

    return fkine(S, M, joints)
                    

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

    quat = get_quaternions(rot_matrix)

    result.position.x = ee_pos[0]
    result.position.y = ee_pos[1]
    result.position.z = ee_pos[2]
    result.orientation.x = quat[0]
    result.orientation.y = quat[1]
    result.orientation.z = quat[2]
    result.orientation.w = quat[3]

def main():

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
        main()
    except rospy.ROSInterruptException:
        pass