#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from trajectory_planner.msg import ApriltagData

last_tag_status = False

position_offset_x = 0.0879895132931
position_offset_y = 0.000217049942842
position_offset_z = 0.925964526065

result = Point()
initial_position = Point()

result.x = position_offset_x
result.y = position_offset_y
result.z = position_offset_z

def calibrate_init_pose(zero_tag_pose):
    global initial_position

    log_msg = "Recorded initial position"
    rospy.loginfo(log_msg)
    
    initial_position = zero_tag_pose.tag_pose.pose.position


def calc_desired_position(tag_position):
    delta_x = tag_position.x - initial_position.x
    delta_y = tag_position.y - initial_position.y
    delta_z = tag_position.z - initial_position.z

    result.x = position_offset_x + delta_x
    result.y = position_offset_y + delta_y
    result.z = position_offset_z + delta_z


def callback(msg):
    global last_tag_status

    if msg.on_screen.data != last_tag_status:
        last_tag_status = msg.on_screen.data
    
        if msg.on_screen.data:
            calibrate_init_pose(msg)
    elif msg.on_screen.data:
        calc_desired_position(msg.tag_pose.pose.position)
    else:
        pass
    
def main():
    rospy.init_node("desired_pose_pub")

    sub = rospy.Subscriber("/tag_in_world", ApriltagData, callback)
    pub = rospy.Publisher("/desired_point", Point, queue_size=1)

    r = rospy.Rate(5.0)

    while not rospy.is_shutdown():

        pub.publish(result)
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass