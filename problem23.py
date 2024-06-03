#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def parabolic_blend_trajectory(t, t_f, theta_start, theta_end):
    a0 = theta_start
    a1 = 0  # initial velocity is zero
    a2 = 3 * (theta_end - theta_start) / t_f**2
    a3 = -2 * (theta_end - theta_start) / t_f**3

    theta_t = a0 + a1 * t + a2 * t**2 + a3 * t**3
    theta_dot_t = a1 + 2 * a2 * t + 3 * a3 * t**2
    return theta_t, theta_dot_t

def main():
    rospy.init_node('robot_arm_trajectory_controller', anonymous=True)
    pub1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)

    rospy.sleep(1)

    start_angles = [0.0, 0.0]
    end_angles = [67.0 * (np.pi / 180.0), -45.0 * (np.pi / 180.0)]

    max_velocities = [9.0 * (np.pi / 180.0), 11.0 * (np.pi / 180.0)]

    delta_angles = [end_angles[i] - start_angles[i] for i in range(2)]
    durations = [abs(delta_angles[i] / max_velocities[i]) for i in range(2)]
    duration = max(durations)

    rate = rospy.Rate(10)
    steps = int(duration * 10)

    bag = rosbag.Bag('joint_trajectories.bag', 'w')

    try:
        for i in range(steps):
            t = i / float(steps) * duration
            q1, q1_dot = parabolic_blend_trajectory(t, duration, start_angles[0], end_angles[0])
            q2, q2_dot = parabolic_blend_trajectory(t, duration, start_angles[1], end_angles[1])

            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['joint_q1', 'joint_q2']
            joint_state.position = [q1, q2]
            joint_state.velocity = [q1_dot, q2_dot]

            pub1.publish(Float64(q1))
            pub2.publish(Float64(q2))
            bag.write('/joint_states', joint_state)

            rospy.loginfo('Published joint state: {}'.format(joint_state))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        bag.close()

    # Plot desired trajectories
    plot_trajectories('joint_trajectories.bag', duration, start_angles, end_angles)

def plot_trajectories(bag_file, duration, start_angles, end_angles):
    times = []
    positions_q1 = []
    positions_q2 = []
    velocities_q1 = []
    velocities_q2 = []

    desired_times = np.linspace(0, duration, 100)
    desired_positions_q1, desired_velocities_q1 = parabolic_blend_trajectory(desired_times, duration, start_angles[0], end_angles[0])
    desired_positions_q2, desired_velocities_q2 = parabolic_blend_trajectory(desired_times, duration, start_angles[1], end_angles[1])

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
    plt.plot(desired_times, desired_positions_q1, 'r--', label='Desired q1 position')
    plt.plot(desired_times, desired_positions_q2, 'b--', label='Desired q2 position')
    plt.plot(times, positions_q1, 'r-', label='Actual q1 position')
    plt.plot(times, positions_q2, 'b-', label='Actual q2 position')
    plt.xlabel('Time [s]')
    plt.ylabel('Position [rad]')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(desired_times, desired_velocities_q1, 'r--', label='Desired q1 velocity')
    plt.plot(desired_times, desired_velocities_q2, 'b--', label='Desired q2 velocity')
    plt.plot(times, velocities_q1, 'r-', label='Actual q1 velocity')
    plt.plot(times, velocities_q2, 'b-', label='Actual q2 velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [rad/s]')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
