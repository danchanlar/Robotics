#!usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Αρχικές και τελικές συνθήκες
q0 = np.array([0, 0, 0])
qf = np.array([5, 3, 1.54])
tf = 10

def cubic_polynomial(q0, qf, tf):
    a0 = q0
    a1 = np.zeros_like(q0)
    a2 = 3 * (qf - q0) / tf**2
    a3 = -2 * (qf - q0) / tf**3
    return a0, a1, a2, a3

a0, a1, a2, a3 = cubic_polynomial(q0, qf, tf)

t = np.linspace(0, tf, 100).reshape(-1, 1)  # Reshape t to be a column vector
q = a0 + a1 * t + a2 * t**2 + a3 * t**3
dq = a1 + 2 * a2 * t + 3 * a3 * t**2

# Καταγραφή των πραγματικών τροχιών
actual_q = []
actual_dq = []

current_position = np.array([0, 0, 0])

def odom_callback(data):
    global current_position
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    current_position = np.array([position.x, position.y, yaw])

def move_robot(a0, a1, a2, a3, duration):
    rospy.init_node('move_robot', anonymous=True)
    pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = (rospy.Time.now() - start_time).to_sec()
        if current_time > duration:
            break

        q = a0 + a1 * current_time + a2 * current_time**2 + a3 * current_time**3
        dq = a1 + 2 * a2 * current_time + 3 * a3 * current_time**2

        actual_q.append(current_position)
        actual_dq.append(dq)

        twist = Twist()
        twist.linear.x = dq[0]
        twist.linear.y = dq[1]
        twist.angular.z = dq[2]
        pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot(a0, a1, a2, a3, tf)
    except rospy.ROSInterruptException:
        pass

# Μετατροπή των πραγματικών τροχιών σε numpy arrays
actual_q = np.array(actual_q)
actual_dq = np.array(actual_dq)
actual_t = np.linspace(0, tf, len(actual_q)).reshape(-1, 1)

plt.figure(figsize=(12, 12))

# Επιθυμητές Θέσεις
plt.subplot(4, 1, 1)
plt.plot(t, q)
plt.title('Επιθυμητές Θέσεις (q) σε συνάρτηση του χρόνου')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Θέση (m, rad)')
plt.legend(['x', 'y', 'θ'])

# Πραγματικές Θέσεις
plt.subplot(4, 1, 2)
if len(actual_q) > 0:
    plt.plot(actual_t, actual_q)
else:
    plt.plot([0, tf], [q0, qf], 'r--')  # Fallback line if no data
plt.title('Πραγματικές Θέσεις (q) σε συνάρτηση του χρόνου')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Θέση (m, rad)')
plt.legend(['x', 'y', 'θ'])

# Επιθυμητές Ταχύτητες
plt.subplot(4, 1, 3)
plt.plot(t, dq)
plt.title('Επιθυμητές Ταχύτητες (dq) σε συνάρτηση του χρόνου')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Ταχύτητα (m/s, rad/s)')
plt.legend(['dx', 'dy', 'dθ'])

# Πραγματικές Ταχύτητες
plt.subplot(4, 1, 4)
if len(actual_dq) > 0:
    plt.plot(actual_t, actual_dq)
else:
    plt.plot([0, tf], [dq[0], dq[-1]], 'r--')  # Fallback line if no data
plt.title('Πραγματικές Ταχύτητες (dq) σε συνάρτηση του χρόνου')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Ταχύτητα (m/s, rad/s)')
plt.legend(['dx', 'dy', 'dθ'])

plt.tight_layout()
plt.show()
