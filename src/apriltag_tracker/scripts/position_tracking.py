#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from kinematics.srv import *

def get_joint_values(position):
    rospy.wait_for_service('/inverse_kinematics')
    try:
        inv_kin_call = rospy.ServiceProxy('/inverse_kinematics', InvKin)
        inv_kin_request = InvKinRequest()
        inv_kin_request.target_pose = position
        res = inv_kin_call(inv_kin_request)
    
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def write_position(joint_vals):
    rospy.wait_for_service('/trajectory_controller')
    try:
        joint_call = rospy.ServiceProxy('/trajectory_controller', MoveJoint)
        joint_req = MoveJointRequest()
        joint_req.joint_set_points = joint_vals.joint_vals
        joint_call(joint_req)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(desired_point):

    joint_vals = get_joint_values(desired_point)
    write_position(joint_vals)

    
def main():
    rospy.init_node("position_tracker")
    init_msg = "Ready to follow an Apriltag!"
    rospy.loginfo(init_msg)
    sub = rospy.Subscriber("/desired_point", Point, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass