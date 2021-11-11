#!/usr/bin/env python
import rospy
import tf
import tf2_ros
# import sympy as sym
from gazebo_msgs.srv import *
from geometry_msgs.msg import PoseStamped

result = PoseStamped()

def callback(tf_listener_):
    
    if tf_listener_.frameExists("/panda_link0") and tf_listener_.frameExists("/panda_hand"):
        t = tf_listener_.getLatestCommonTime("/panda_link0", "/panda_hand")
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.stamp = rospy.Time.now()
        p1.header.frame_id = "panda_hand"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = tf_listener_.transformPose("/panda_link0", p1)
        result = p_in_base
        print("Position of the fingertip in the robot base:")
        print(p_in_base)


def main():
    rospy.init_node("transforms")

    tf_listener_ = tf.TransformListener()

    
    pub = rospy.Publisher("/panda_arm/ee_in_world", PoseStamped, queue_size=1)

    r = rospy.Rate(60.0)

    while not rospy.is_shutdown():

        callback(tf_listener_)
        # rospy.loginfo(result)

        pub.publish(result)

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass