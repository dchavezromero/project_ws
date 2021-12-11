#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from kinematics.srv import *

def get_joint_values(position):
    # rospy.wait_for_service('/inverse_kinematics')
    try:
        inv_kin_call = rospy.ServiceProxy('/inverse_kinematics', InvKin)
        # joint_data = inv_kin_call(joint)
        inv_kin_request = InvKinRequest()
        inv_kin_request.target_pose = position
        res = inv_kin_call(inv_kin_request)
    
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def write_position(joint_vals):
    rospy.wait_for_service('/position_controller')

    try:
        joint_call = rospy.ServiceProxy('/position_controller', MoveJoint)
        joint_req = MoveJointRequest()
        joint_req.joint_set_points = joint_vals.joint_vals
        joint_call(joint_req)
        # rospy.loginfo("Attempting to drive joints to calculated joint values.")

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(desired_point):
    print("attempting to do inv kin for point:")
    print(desired_point)

    joint_vals = get_joint_values(desired_point)

    # print(joint_vals)
    write_position(joint_vals)

    # print(joint_vals)
    
def main():
    rospy.init_node("position_tracker")
    sub = rospy.Subscriber("/desired_point", Point, callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass