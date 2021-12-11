#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_planner.msg import ApriltagData

last_tag_status = False

result = PoseStamped()
initial_pose = PoseStamped()


def calibrate_init_pose(zero_tag_pose):
    global initial_pose

    log_msg = "Time calibrate pose offset"
    rospy.loginfo(log_msg)
    initial_pose = zero_tag_pose.tag_pose

    # print(initial_pose)


def callback(msg):
    global last_tag_status

    if msg.on_screen.data != last_tag_status:
        last_tag_status = msg.on_screen.data
    
        if msg.on_screen.data:
            calibrate_init_pose(msg)

    # if not logged_init_pose:
    #     initial_pose = msg
    #     logged_init_pose = True
    #     print("set init pose")


def main():
    rospy.init_node("desired_pose_pub")

    sub = rospy.Subscriber("/tag_in_world", ApriltagData, callback)
    # pub = rospy.Publisher("/tag_in_world", PoseStamped, queue_size=1)

    r = rospy.Rate(60.0)

    while not rospy.is_shutdown():

        # pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass