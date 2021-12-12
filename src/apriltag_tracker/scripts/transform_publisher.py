#!/usr/bin/env python

import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_tracker.msg import ApriltagData

result = ApriltagData()

def callback(msg, tf_listener_):

    position = None
    quaternion = None

    if(len(msg.detections) == 1):

        while position is None and quaternion is None:

            try:
                tag_num = msg.detections[0].id[0]

                tag_id = '/tag_'+ str(tag_num)
                position, quaternion = tf_listener_.lookupTransform("/world", tag_id, rospy.Time())

                result.tag_pose.pose.position.x = position[0]
                result.tag_pose.pose.position.y = position[1]
                result.tag_pose.pose.position.z = position[2]
                result.tag_pose.pose.orientation.x = quaternion[0]
                result.tag_pose.pose.orientation.y = quaternion[1]
                result.tag_pose.pose.orientation.z = quaternion[2]
                result.tag_pose.pose.orientation.w = quaternion[3]
                result.tag_pose.header.frame_id = "world"
                result.tag_pose.header.stamp = rospy.Time.now()

                result.on_screen.data = True
            except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
                error_msg = "Some TF issue..."
                rospy.logwarn_throttle_identical(1, error_msg)
                pass
        
    elif(len(msg.detections) > 1):
        error_msg = "More than 1 apriltag on-screen!"
        result.on_screen.data = False
        rospy.logerr_throttle_identical(1, error_msg)
    else:
        warning_msg = "No apriltags on-screen!"
        result.on_screen.data = False
        rospy.logwarn_throttle_identical(1, warning_msg)

def main():
    rospy.init_node("tag_to_world_transform_pub")

    tf_listener_ = tf.TransformListener()

    sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback, tf_listener_)
    tag_data_world = rospy.Publisher("/tag_in_world", ApriltagData, queue_size=1)

    r = rospy.Rate(60.0)

    while not rospy.is_shutdown():

        tag_data_world.publish(result)
        # tag_status.publish(single_tag_on_screen)

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass