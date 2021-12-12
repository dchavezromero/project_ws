
import rospy
import math
import sympy as sym
from gazebo_msgs.srv import *
from sympy.vector import CoordSys3D
import modern_robotics as mr
import numpy as np

class CommonFunctions(object):
    def __init__(self):
        # Link length of values (meters)
        L0 = 0.088
        L1 = 0.333
        L2 = 0.316
        L3 = 0.384
        L4 = 0.107
        L5 = 0.0825

        self.S_space = sym.Matrix([[0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, -1, 0, -1, 0],
                        [1, 0, 1, 0, 1, 0, -1],
                        [0, -L1, 0, L1+L2, 0, L1+L2+L3, 0],
                        [0, 0, 0, 0, 0, 0, L0],
                        [0, 0, 0, -L5, 0, 0, 0]])

        self.S_body = sym.Matrix([[0, 0,-1, 0, -L0, 0], 
                                [0,-1, 0, L2+L3-L4, 0, L0], 
                                [0, 0,-1, 0, -L0, 0],
                                [0, 1, 0, L4-L3, 0, L5-L0],
                                [0, 0,-1, 0, -L0, 0], 
                                [0, 1, 0, L4, 0, -L0], 
                                [0, 0, 1, 0, 0, 0]])

        R = sym.Matrix([[1, 0, 0],
                        [0, -1, 0],
                        [0, 0, -1]])

        p = sym.Matrix([L0, 0, L1+L2+L3-L4])

        top = sym.Matrix(sym.BlockMatrix([[R, p]]))
        bot = sym.Matrix([[0, 0, 0, 1]])

        self.M = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

    def jacoba(self,S_b,q, T):

        R = T[0:3, 0:3]

        J_b = sym.Matrix(mr.JacobianBody(np.array(S_b).astype(np.float64),np.array(q).astype(np.float64)))

        Jb_v = J_b[3:6,:]

        self.J_a = R*Jb_v

        return self.J_a

    def norm(self, vector):
        C = CoordSys3D('C')
        v = vector[0]*C.i + vector[1]*C.j + vector[2]*C.k
        norm = math.sqrt(v.dot(v))

        return norm

    def get_position(self, joint):
        rospy.wait_for_service('/gazebo/get_joint_properties')
        try:
            joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
            joint_data = joint_call(joint)
            position = joint_data.position[0]
            return position
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_current_joints_vals(self):
        panda_joint1 = self.get_position('panda_joint1')
        panda_joint2 = self.get_position('panda_joint2')
        panda_joint3 = self.get_position('panda_joint3')
        panda_joint4 = self.get_position('panda_joint4')
        panda_joint5 = self.get_position('panda_joint5')
        panda_joint6 = self.get_position('panda_joint6')
        panda_joint7 = self.get_position('panda_joint7')

        self.joint_angles = sym.Matrix([panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7])

        return self.joint_angles

    def check_limits(self, joint_angles):
        self.response = False

        theta1 = joint_angles[0]
        theta2 = joint_angles[1]
        theta3 = joint_angles[2]
        theta4 = joint_angles[3]
        theta5 = joint_angles[4]
        theta6 = joint_angles[5]
        theta7 = joint_angles[6]

        if(len(joint_angles) > 7):
            finger1 = joint_angles[7]
            finger2 = joint_angles[8]
        else:
            finger1 = 0
            finger2 = 0

        if (rospy.has_param('/panda_arm/panda_joint1_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint1_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint2_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint2_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint3_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint3_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint4_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint4_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint5_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint5_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint6_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint6_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_joint7_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_joint7_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_finger1_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_finger1_joint_limits/upper') and
        rospy.has_param('/panda_arm/panda_finger2_joint_limits/lower') and
        rospy.has_param('/panda_arm/panda_finger2_joint_limits/upper')):

            theta1_lower_limit = rospy.get_param('/panda_arm/panda_joint1_joint_limits/lower')
            theta2_lower_limit = rospy.get_param('/panda_arm/panda_joint2_joint_limits/lower')
            theta3_lower_limit = rospy.get_param('/panda_arm/panda_joint3_joint_limits/lower')
            theta4_lower_limit = rospy.get_param('/panda_arm/panda_joint4_joint_limits/lower')
            theta5_lower_limit = rospy.get_param('/panda_arm/panda_joint5_joint_limits/lower')
            theta6_lower_limit = rospy.get_param('/panda_arm/panda_joint6_joint_limits/lower')
            theta7_lower_limit = rospy.get_param('/panda_arm/panda_joint7_joint_limits/lower')
            finger1_lower_limit = rospy.get_param('/panda_arm/panda_finger1_joint_limits/lower')
            finger2_lower_limit = rospy.get_param('/panda_arm/panda_finger2_joint_limits/lower')
            theta1_upper_limit = rospy.get_param('/panda_arm/panda_joint1_joint_limits/upper')
            theta2_upper_limit = rospy.get_param('/panda_arm/panda_joint2_joint_limits/upper')
            theta3_upper_limit = rospy.get_param('/panda_arm/panda_joint3_joint_limits/upper')
            theta4_upper_limit = rospy.get_param('/panda_arm/panda_joint4_joint_limits/upper')
            theta5_upper_limit = rospy.get_param('/panda_arm/panda_joint5_joint_limits/upper')
            theta6_upper_limit = rospy.get_param('/panda_arm/panda_joint6_joint_limits/upper')
            theta7_upper_limit = rospy.get_param('/panda_arm/panda_joint7_joint_limits/upper')
            finger1_upper_limit = rospy.get_param('/panda_arm/panda_finger1_joint_limits/upper')
            finger2_upper_limit = rospy.get_param('/panda_arm/panda_finger2_joint_limits/upper')

            
            if ((theta1 <= theta1_upper_limit and theta1 >= theta1_lower_limit) 
            and (theta2 <= theta2_upper_limit and theta2 >= theta2_lower_limit) 
            and (theta3 <= theta3_upper_limit and theta3 >= theta3_lower_limit) 
            and (theta4 <= theta4_upper_limit and theta4 >= theta4_lower_limit) 
            and (theta5 <= theta5_upper_limit and theta5 >= theta5_lower_limit) 
            and (theta6 <= theta6_upper_limit and theta6 >= theta6_lower_limit) 
            and (theta7 <= theta7_upper_limit and theta7 >= theta7_lower_limit)
            and (finger1 <= finger1_upper_limit and finger1 >= finger1_lower_limit) 
            and (finger2 <= finger2_upper_limit and finger2 >= finger2_lower_limit)):

                self.response = True

            return self.response
        else:
            err_msg = "Joint param limits not loaded"
            rospy.logerr(err_msg)
            return self.response

if __name__=='main':
    common_object = CommonFunctions()
