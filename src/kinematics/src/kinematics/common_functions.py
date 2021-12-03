
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

if __name__=='main':
    common_object = CommonFunctions()
