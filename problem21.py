#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('robot_arm_controller', anonymous=True)
    pub1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)

    rospy.sleep(1)

    # Initial and final angles in radians
    start_angles = [0.0, 0.0]
    end_angles = [25.0 * (np.pi / 180.0), -30.0 * (np.pi / 180.0)]

    # Duration of the movement in seconds and publishing frequency in Hz
    duration = 10
    rate = rospy.Rate(10)  # 10 Hz
    steps = duration * 10
    dt = 1.0 / 10  # time step duration

    # Open the rosbag file
    bag = rosbag.Bag('joint_states.bag', 'w')

    try:
        previous_angles = start_angles
        for i in range(steps):
            t = i / float(steps)
            current_angles = [
                start_angles[0] + t * (end_angles[0] - start_angles[0]),
                start_angles[1] + t * (end_angles[1] - start_angles[1])
            ]
            velocities = [
                (current_angles[0] - previous_angles[0]) / dt,
                (current_angles[1] - previous_angles[1]) / dt
            ]

            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['joint_q1', 'joint_q2']
            joint_state.position = current_angles
            joint_state.velocity = velocities

            # Publish the data
            pub1.publish(Float64(current_angles[0]))
            pub2.publish(Float64(current_angles[1]))

            # Write to rosbag
            bag.write('/joint_states', joint_state)

            # Update previous_angles
            previous_angles = current_angles

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        bag.close()

def plot_trajectories(bag_file):
    times = []
    positions_q1 = []
    positions_q2 = []
    velocities_q1 = []
    velocities_q2 = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            times.append(t.to_sec())
            positions_q1.append(msg.position[0])
            positions_q2.append(msg.position[1])
            velocities_q1.append(msg.velocity[0])
            velocities_q2.append(msg.velocity[1])

    times = np.array(times) - times[0]

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(times, positions_q1, label='q1 position')
    plt.plot(times, positions_q2, label='q2 position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(times, velocities_q1, label='q1 velocity')
    plt.plot(times, velocities_q2, label='q2 velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        main()
        plot_trajectories('joint_states.bag')
    except rospy.ROSInterruptException:
        pass
