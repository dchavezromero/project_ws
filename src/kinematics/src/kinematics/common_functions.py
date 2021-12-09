
import rospy
import math
import sympy as sym
from gazebo_msgs.srv import *
from sympy.vector import CoordSys3D

class CommonFunctions(object):
    def __init__(self):
        # Link length of values (meters)
        L0 = 0.088
        L1 = 0.333
        L2 = 0.316
        L3 = 0.384
        L4 = 0.107
        L5 = 0.0825

        self.S = sym.Matrix([[0, 0, 0, 0, 0, 0, 0],
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

        self.M = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

    def euler_to_quaternion(self, roll, pitch, yaw):
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

        self.quaternions = qw, qx, qy, qz

        # rospy.loginfo(quaternions)

        return self.quaternions

    def get_quaternions(self, rot_matrix):
        roll = math.atan2(rot_matrix[7], rot_matrix[8])
        pitch = math.atan2(-rot_matrix[6], math.sqrt(pow(rot_matrix[7], 2) + pow(rot_matrix[8], 2)))
        yaw = math.atan2(rot_matrix[3], rot_matrix[0])

        # euler_angles = roll, pitch, yaw

        # rospy.loginfo(euler_angles)

        self.quaternions = self.euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))
        return self.quaternions

    def skew(self, v):
        if len(v) == 3:
            self.S_matrix = sym.Matrix([[0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        else:
            rospy.logerr("argument must be a 3-vector")

        return self.S_matrix

    def axis_angle_2_rot(self, omega, theta):
        omega_ss = self.skew(omega)
        self.R = sym.eye(3) + math.sin(theta) * omega_ss + (1-math.cos(theta)) * omega_ss**2

        return self.R

    def twist_2_ht(self, S, theta):
        w = S[0:3,:]
        v = S[3:6,:]

        R = self.axis_angle_2_rot(w,theta)

        s_skew = self.skew(w)

        col = ((sym.eye(3)*theta) + (1-math.cos(theta))*s_skew + (theta-math.sin(theta))*(s_skew**2))*v

        top = sym.Matrix(sym.BlockMatrix([[R, col]]))
        bot = sym.Matrix([[0, 0, 0, 1]])

        self.T = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

        return self.T

    def fkine(self, S, M, q, frame):
        self.M = M

        n_joints = len(q)

        T_n = sym.eye(4)

        for n in range(n_joints):
            T_n = T_n*self.twist_2_ht(S[:,n], q[n])

        if frame == 'space':
            T_n = T_n*self.M
        elif frame == 'body':
            T_n = self.M*T_n

        self.T_n = T_n

        return self.T_n

    def adjoint(self, T):
        R = T[0:3,0:3]
        p = T[0:3, 3]

        skew_p = self.skew(p)

        top = sym.Matrix(sym.BlockMatrix([[R, sym.zeros(3)]]))
        bot = sym.Matrix(sym.BlockMatrix([[skew_p*R, R]]))

        self.adT = sym.Matrix(sym.BlockMatrix([[top], [bot]]))

        return self.adT

    def twist_space_2_body(self,V_s,T):
        adT = self.adjoint(T)
        self.V_b = (adT.inv())*V_s

        return self.V_b

    def jacob0(self, S, q):
        n_joints = len(q)
        J = sym.zeros(6, n_joints)
        
        T_n = sym.eye(4)

        for n in range(n_joints):
            T_n = T_n*self.twist_2_ht(S[:,n], q[n])
            J[:, n] = self.adjoint(T_n)*S[:,n]

        self.J = J

        return self.J

    def jacobe(self, S, M, q):
        J_s = self.jacob0(S,q)
        T = self.fkine(S,M,q,'space')

        self.J_b = self.twist_space_2_body(J_s, T)

        return self.J_b

    def jacoba(self,S,M,q):
        T = self.fkine(S,M,q,'space')
        R = T[0:3, 0:3]
        J_b = self.jacobe(S,M,q)
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


        # print(type(theta1))
        # print((theta1 <= 2.8973 and theta1 >= -2.8973))

        # Maybe change to joint limits to be referenced from xacro file
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

                # print(theta1, " is within joint 1 angle limits -> LOWER: ", theta1_lower_limit, " | UPPER: ", theta1_upper_limit)
                # print(theta2, " is within joint 2 angle limits -> LOWER: ", theta2_lower_limit, " | UPPER: ", theta2_upper_limit)
                # print(theta3, " is within joint 3 angle limits -> LOWER: ", theta3_lower_limit, " | UPPER: ", theta3_upper_limit)
                # print(theta4, " is within joint 4 angle limits -> LOWER: ", theta4_lower_limit, " | UPPER: ", theta4_upper_limit)
                # print(theta5, " is within joint 5 angle limits -> LOWER: ", theta5_lower_limit, " | UPPER: ", theta5_upper_limit)
                # print(theta6, " is within joint 6 angle limits -> LOWER: ", theta6_lower_limit, " | UPPER: ", theta6_upper_limit)
                # print(theta7, " is within joint 7 angle limits -> LOWER: ", theta7_lower_limit, " | UPPER: ", theta7_upper_limit)

                self.response = True
            else:
                if not (theta1 <= theta1_upper_limit and theta1 >= theta1_lower_limit):
                    print(theta1, " is NOT within joint 1 angle limits -> LOWER: ", theta1_lower_limit, " | UPPER: ", theta1_upper_limit)
                if not (theta2 <= theta2_upper_limit and theta2 >= theta2_lower_limit):
                    print(theta2, " is NOT within joint 2 angle limits -> LOWER: ", theta2_lower_limit, " | UPPER: ", theta2_upper_limit)
                if not (theta3 <= theta3_upper_limit and theta3 >= theta3_lower_limit):
                    print(theta3, " is NOT within joint 3 angle limits -> LOWER: ", theta3_lower_limit, " | UPPER: ", theta3_upper_limit)
                if not (theta4 <= theta4_upper_limit and theta4 >= theta4_lower_limit):
                    print(theta4, " is NOT within joint 4 angle limits -> LOWER: ", theta4_lower_limit, " | UPPER: ", theta4_upper_limit)
                if not (theta5 <= theta5_upper_limit and theta5 >= theta5_lower_limit):
                    print(theta5, " is NOT within joint 5 angle limits -> LOWER: ", theta5_lower_limit, " | UPPER: ", theta5_upper_limit)
                if not (theta6 <= theta6_upper_limit and theta6 >= theta6_lower_limit):
                    print(theta6, " is NOT within joint 6 angle limits -> LOWER: ", theta6_lower_limit, " | UPPER: ", theta6_upper_limit)
                if not (theta7 <= theta7_upper_limit and theta7 >= theta7_lower_limit):
                    print(theta7, " is NOT within joint 7 angle limits -> LOWER: ", theta7_lower_limit, " | UPPER: ", theta7_upper_limit)

                # pub_setPoint.publish(set_points)

            return self.response
        else:
            rospy.logerr("Joint param limits not loaded")
            return self.response

if __name__=='main':
    common_object = CommonFunctions()
