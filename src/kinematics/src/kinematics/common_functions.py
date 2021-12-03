
import rospy
import math
import sympy as sym

class CommonFunctions(object):
    def __init__(self):
        pass

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
            self.S = sym.Matrix([[0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]])
        else:
            rospy.logerr("argument must be a 3-vector")

        return self.S

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



if __name__=='main':
    common_object = CommonFunctions()
