#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


arm_trajectory = JointTrajectory()

arm_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
"panda_joint5", "panda_joint6", "panda_joint7"]


def send_trajectories():
        print("sending trajectories!")
        goal_positions = [0, -0.6, 0, -1.5, 0, 2.4, 0]
        arm_trajectory.joint_names = arm_names
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = goal_positions
        arm_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
        arm_trajectory.points[0].accelerations = [0, 0, 0, 0, 0, 0, 0]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3)
        rospy.sleep(1)
        

def main():

    rospy.init_node("test_traj")
    
    rospy.loginfo("Initializing Joint Trajectory Node")

    pub = rospy.Publisher('/panda_arm/arm_controller/command', JointTrajectory, queue_size=1)

    r = rospy.Rate(20.0)

    send_trajectories()

    while not rospy.is_shutdown():

        # rospy.loginfo("test")

        pub.publish(arm_trajectory)

        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass