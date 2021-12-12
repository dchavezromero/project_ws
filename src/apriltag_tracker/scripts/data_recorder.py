#!/usr/bin/env python
import rospy
import csv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

def record_joint_data(time, joint_pos_data, joint_vel_data, joint_eff_data):

    with open('joint_position_data.csv', 'a') as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow((time, joint_pos_data[2], joint_pos_data[3], joint_pos_data[4], joint_pos_data[5], joint_pos_data[6], joint_pos_data[7], joint_pos_data[8]))

    with open('joint_velocity_data.csv', 'a') as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow((time, joint_vel_data[2], joint_vel_data[3], joint_vel_data[4], joint_vel_data[5], joint_vel_data[6], joint_vel_data[7], joint_vel_data[8]))

    with open('joint_effort_data.csv', 'a') as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow((time, joint_eff_data[2], joint_eff_data[3], joint_eff_data[4], joint_eff_data[5], joint_eff_data[6], joint_eff_data[7], joint_eff_data[8]))


def record_ee_data(x,y,z):

    with open('ee_position_data.csv', 'a') as csv_file:

        writer = csv.writer(csv_file)
        writer.writerow((x, y, z))


def joint_data_callback(current_joint_states):
    current_time = current_joint_states.header.stamp

    joint_positions = current_joint_states.position
    joint_velocities = current_joint_states.velocity
    joint_efforts = current_joint_states.effort
    
    record_joint_data(current_time, joint_positions, joint_velocities, joint_efforts)

def ee_data_callback(ee_pose):
    x = ee_pose.position.x
    y = ee_pose.position.y
    z = ee_pose.position.z

    record_ee_data(x,y,z)    

def main():
    rospy.init_node("data_recorder")
    init_msg = "Ready to record data!"
    rospy.loginfo(init_msg)
    joint_sub = rospy.Subscriber("/panda_arm/joint_states", JointState, joint_data_callback)
    ee_sub = rospy.Subscriber("/panda_arm/ee_pose", Pose, ee_data_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass