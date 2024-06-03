#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def move_robot(linear_speed, angular_speed, duration, bag):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time).to_sec() < duration:
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        pub.publish(twist_msg)
        bag.write('/cmd_vel', twist_msg)
        rate.sleep()

def plot_trajectories(bag_file):
    times = []
    linear_velocities = []
    angular_velocities = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
            times.append(t.to_sec())
            linear_velocities.append(msg.linear.x)
            angular_velocities.append(msg.angular.z)

    times = np.array(times) - times[0]

    # Calculate displacements
    linear_displacement = np.cumsum(linear_velocities) / 10  # Since rate is 10 Hz
    angular_displacement = np.cumsum(angular_velocities) / 10  # Since rate is 10 Hz

    plt.figure()

    # Plot displacements
    plt.subplot(2, 1, 1)
    plt.plot(times, linear_displacement, label='Linear Displacement')
    plt.plot(times, angular_displacement, label='Angular Displacement')
    plt.xlabel('Time [s]')
    plt.ylabel('Displacement [m/rad]')
    plt.legend()

    # Plot velocities
    plt.subplot(2, 1, 2)
    plt.plot(times, linear_velocities, label='Linear Velocity')
    plt.plot(times, angular_velocities, label='Angular Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s or rad/s]')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        rospy.init_node('move_robot', anonymous=True)

        # Initialize rosbag for recording
        bag = rosbag.Bag('robot_trajectory.bag', 'w')

        # Move the robot with the specified speeds and durations
        move_robot(0, -0.261799, 10, bag)  # [0, -15 degrees/sec], 10 seconds
        move_robot(0.1, 0.174533, 15, bag)  # [0.1 m/s, 10 degrees/sec], 15 seconds

    except rospy.ROSInterruptException:
        pass
    finally:
        if not bag._file.closed:
            bag.close()

        # Plot the recorded trajectories
        plot_trajectories('robot_trajectory.bag')
