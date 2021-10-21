#!/usr/bin/env python
import rospy
import csv
import time
from gazebo_msgs.srv import *
from kinematics.msg import joint_angles 

joint_names = 'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'

Kp_vals = [50, 50, 10, 10, 10, 10, 10]
Kd_vals = [5, 1, 5, 7, 7, 5, 5]

# 10Hz frequency
ros_rate = 10.0

# Loop execution rate in seconds (1/freq)
sampling_rate = (1/ros_rate) 
last_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
last_set_points = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Record data every 15 seconds
# record_time_interval = 15

# start_time = 0
# timer_started = False
# times = []
# set_points = []
# curr_points = []

# def reset_timer():
#     global times, set_points, curr_points, timer_started

#     times = []
#     set_points = []
#     curr_points = []
#     timer_started = False

# def record_data(time, set_point, curr_point):

#     with open('position_controller_data.csv', 'wb') as csv_file:

#         writer = csv.writer(csv_file, delimiter=',')

#         for i in range(len(time)):
#             writer.writerow((time[i], set_point[i], curr_point[i]))


def write_effort(effort, duration_sec, joint_name):
    rospy.wait_for_service('/gazebo/apply_joint_effort')

    try:
        joint_call = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        joint_req = ApplyJointEffortRequest()
        joint_req.joint_name = joint_name
        joint_req.effort = effort
        joint_req.duration.nsecs = duration_sec * 100000000 #Convert sampling rate to nanoseconds
        res = joint_call(joint_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def do_pd_control(set_point, curr_point, joint_name):
    global last_positions, last_set_points #, timer_started, times, set_points, curr_points, start_time

    # Start timer for recording data
    # if not timer_started:
    #     start_time = time.time()
    #     timer_started = True

    # Calculate errors
    position_err = set_point - curr_point


    if (joint_name is 'panda_joint1'):

        derivative_err = (last_positions[0] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[0] = curr_point
        last_set_points[0] = set_point

        # PD output
        effort = position_err*Kp_vals[0] + derivative_err*Kd_vals[0]
        rospy.loginfo(effort)
    elif (joint_name is 'panda_joint2'):

        derivative_err = (last_positions[1] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[1] = curr_point
        last_set_points[1] = set_point

        # PD output
        effort = position_err*Kp_vals[1] + derivative_err*Kd_vals[1]
        # rospy.loginfo(effort)
    elif (joint_name is 'panda_joint3'):

        derivative_err = (last_positions[2] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[2] = curr_point
        last_set_points[2] = set_point

        # PD output
        effort = position_err*Kp_vals[2] + derivative_err*Kd_vals[2]
    elif (joint_name is 'panda_joint4'):

        derivative_err = (last_positions[3] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[3] = curr_point
        last_set_points[3] = set_point

        # PD output
        effort = position_err*Kp_vals[3] + derivative_err*Kd_vals[3]
    elif (joint_name is 'panda_joint5'):

        derivative_err = (last_positions[4] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[4] = curr_point
        last_set_points[4] = set_point

        # PD output
        effort = position_err*Kp_vals[4] + derivative_err*Kd_vals[4]
    elif (joint_name is 'panda_joint6'):

        derivative_err = (last_positions[5] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[5] = curr_point
        last_set_points[5] = set_point

        # PD output
        effort = position_err*Kp_vals[5] + derivative_err*Kd_vals[5]
    elif (joint_name is 'panda_joint7'):

        derivative_err = (last_positions[6] - curr_point)/sampling_rate

        # Set reference vals
        last_positions[6] = curr_point
        last_set_points[6] = set_point

        # PD output
        effort = position_err*Kp_vals[6] + derivative_err*Kd_vals[6]

    # Append data entries to lists
    # times.append(time.time())
    # set_points.append(set_point)
    # curr_points.append(curr_point)

    # Write effort to joint
    write_effort(effort, sampling_rate, joint_name)

    # If 15 secs have passed, record the data to a CSV file and reset timer params
    # if time.time() > (start_time + record_time_interval):
    #     rospy.loginfo("Recorded data")
    #     record_data(times, set_points, curr_points)
    #     reset_timer()


def callback(msg):
    theta1 = get_position(joint_names[0])
    theta2 = get_position(joint_names[1])
    theta3 = get_position(joint_names[2])
    theta4 = get_position(joint_names[3])
    theta5 = get_position(joint_names[4])
    theta6 = get_position(joint_names[5])
    theta7 = get_position(joint_names[6])

    do_pd_control(msg.theta1, theta1, joint_names[0])
    do_pd_control(msg.theta2, theta2, joint_names[1])
    do_pd_control(msg.theta3, theta3, joint_names[2])
    do_pd_control(msg.theta4, theta4, joint_names[3])
    do_pd_control(msg.theta5, theta5, joint_names[4])
    do_pd_control(msg.theta6, theta6, joint_names[5])
    do_pd_control(msg.theta7, theta7, joint_names[6])

def main():
    rospy.init_node("position_controller")

    sub = rospy.Subscriber("/panda/set_points", joint_angles, callback)
    r = rospy.Rate(ros_rate)

    while not rospy.is_shutdown():
        r.sleep()
        do_pd_control(last_set_points[0], get_position(joint_names[0]), joint_names[0]) # maintain theta1 position
        do_pd_control(last_set_points[1], get_position(joint_names[1]), joint_names[1]) # maintain theta2 position
        do_pd_control(last_set_points[2], get_position(joint_names[2]), joint_names[2]) # maintain theta3 position       
        do_pd_control(last_set_points[3], get_position(joint_names[3]), joint_names[3]) # maintain theta4 position
        do_pd_control(last_set_points[4], get_position(joint_names[4]), joint_names[4]) # maintain theta5 position
        do_pd_control(last_set_points[5], get_position(joint_names[5]), joint_names[5]) # maintain theta6 position  
        do_pd_control(last_set_points[6], get_position(joint_names[6]), joint_names[6]) # maintain theta7 position 

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass