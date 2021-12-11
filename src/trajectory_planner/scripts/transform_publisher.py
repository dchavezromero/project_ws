#!/usr/bin/env python
import rospy
from rospy.core import rospyerr
import tf
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

result = PoseStamped()


def callback(msg, tf_listener_):

    position = None
    quaternion = None

    if(len(msg.detections) == 1):

        while position is None and quaternion is None:

            try:
                tag_num = msg.detections[0].id[0]

                tag_id = '/tag_'+ str(tag_num)
                position, quaternion = tf_listener_.lookupTransform("/world", tag_id, rospy.Time())

                result.pose.position.x = position[0]
                result.pose.position.y = position[1]
                result.pose.position.z = position[2]
                result.pose.orientation.x = quaternion[0]
                result.pose.orientation.y = quaternion[1]
                result.pose.orientation.z = quaternion[2]
                result.pose.orientation.w = quaternion[3]
                result.header.frame_id = "world"
                result.header.stamp = rospy.Time.now()
            except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
                error_msg = "Some TF issue..."
                rospy.logwarn_throttle_identical(1, error_msg)
                pass
        
    elif(len(msg.detections) > 1):
        error_msg = "More than 1 apriltag on-screen!"
        rospy.logerr_throttle_identical(1, error_msg)
    else:
        warning_msg = "No apriltags on-screen!"
        rospy.logwarn_throttle_identical(1, warning_msg)

def main():
    rospy.init_node("transforms")

    tf_listener_ = tf.TransformListener()

    sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback, tf_listener_)
    pub = rospy.Publisher("/tag_in_world", PoseStamped, queue_size=1)

    r = rospy.Rate(60.0)

    while not rospy.is_shutdown():

        pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass