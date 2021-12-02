
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

    def fkine(self, S, M, q):
        self.M = M

        n_joints = len(q)

        self.T_n = sym.eye(4)

        for n in range(n_joints):
            self.T_n = self.T_n*self.twist_2_ht(S[:,n], q[n])

        return self.T_n*self.M

if __name__=='main':
    common_object = CommonFunctions()
