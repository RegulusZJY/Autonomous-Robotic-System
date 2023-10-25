#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2

class ClosedLoopTurtlebotDriving():
    def __init__(self):
        rospy.init_node('turtlebot_close_loop', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)
        # Create a publisher, which talks to TurtleBot3 to move
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Create a subscriber, which subscribes to /odom topic, and callback when received msgs
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel = Twist()
        # Set a publish rate of in 10 Hz, the time interval will be 0.1s
        self.rate = rospy.Rate(10)
        self.counter = 0
        # Store trajectory
        self.trajectory = list()
        # Initialize PID controller
        self.pid_theta = Controller(0.0, 0.0, 0.0)
        # self initial pose
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        # Run the code
        self.run()

    def move_to(self, x, y):
        # Calculate the difference between the target point and the current point
        diff_x, diff_y = x - self.x, y - self.y
        # Calculate the direction vector, that is, the direction pointing towards the target point
        direction_vector = np.array([diff_x, diff_y]) / sqrt(diff_x ** 2 + diff_y ** 2)
        # Calculate the angle between the target point (x, y) and the current position of the robot
        theta = atan2(diff_y, diff_x)
        # Set the PID value to determine the angular velocity
        self.pid_theta.setPID(1.0, 0.0, 0.0)
        # Set the PID angle to the calculated angle
        self.pid_theta.setPoint(theta)
        rospy.loginfo("Set target theta = " + str(theta))

        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            # Limit the angular velocity to a maximum of 0.2
            angular = max(-0.2, min(0.2, angular))
            # If the angular velocity is less than 0.01, stop the turtlebot
            if abs(angular) < 0.01:
                break
            self.vel.linear.x, self.vel.angular.z = 0, angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()
        self.pid_theta.setPoint(theta)

        while not rospy.is_shutdown():
            # Recalculate the difference
            diff_x, diff_y = x - self.x, y - self.y
            vector = np.array([diff_x, diff_y])
            # Calculate the linear velocity in direction_vector
            linear = np.dot(vector, direction_vector)
            # Limit the linear velocity to a maximum of 0.2
            linear = max(-0.2, min(0.2, linear))
            angular = self.pid_theta.update(self.theta)
            # Limit the angular velocity to a maximum of 0.2
            angular = max(-0.2, min(0.2, angular))
            # If the linear velocity is less than 0.01 and the angular velocity is less than 0.01, stop the turtlebot
            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x, self.vel.angular.z = linear, angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()

    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x, self.y])
            rospy.loginfo("odom: x=" + str(self.x) + "; y=" + str(self.y) + "; theta=" + str(self.theta))

    # Plot the trajectory
    def plot_trajectory(self):
        data = np.array(self.trajectory)
        # Save the trajectory to a csv file
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:, 0], data[:, 1])
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.title('Turtlebot Trajectory')
        plt.grid(True)
        plt.show()

    def stop(self):
        self.vel.linear.x, self.vel.angular.z = 0, 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def run(self):
        # Set the target point to travel
        for point in [[1, 0], [1, 1], [0, 1], [0, 0]]:
            self.move_to(point[0], point[1])
            rospy.sleep(0.5)
        self.stop()
        rospy.logwarn("Task Completed")
        self.plot_trajectory()

# Proportional–Integral–Derivative (PID) controller
class Controller:
    '''
    In each term we have
    a coefficient k times the error (integral/derivative).
    Kp, Ki and Kd are the coefficients
    we need to adjust.
    '''
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = 0.0
        self.error = 0.0
        self.Integrator = 0.0
        self.Derivator = 0
    '''
    For the integral and derivative of the error,
    we can use summation instead of integral and subtraction
    instead of derivative.
    '''
    def update(self, current_value):
        # Calculate the error, current value is the actual value of the state
        self.error = self.set_point - current_value
        '''
        If the error is greater than p, subtract 2 pi from the error;
        if the error is less than -pi, add 2 pi to the error.
        This is to ensure that the error is between -pi and pi
        to avoid excessive accumulation of integral terms
        '''
        if self.error > pi:
            self.error -= 2 * pi
        elif self.error < -pi:
            self.error += 2 * pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator += self.error
        self.I_value = self.Integrator * self.Ki
        return self.P_value + self.I_value + self.D_value
    
    # Set the target value point
    def setPoint(self, set_point):
        self.set_point = set_point
        # Store the value of the integral term, initialized to 0.0
        self.Integrator = 0
        # Store the value of the derivative term, initialized to 0.0
        self.Derivator = 0
    # Initialize the PID value
    def setPID(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

if __name__ == '__main__':
    try:
        ClosedLoopTurtlebotDriving()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminated.")
