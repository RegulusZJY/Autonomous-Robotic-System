#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from math import pi
from nav_msgs.msg import Odometry
import tf
import matplotlib.pyplot as plt
import numpy as np

class TurtlebotDriving():
    def __init__(self):
        rospy.init_node("turtlebot_driving", anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)
        # Create a publisher, which talks to TurtleBot3 to move
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # Create a subscriber, which subscribes to /odom topic, and callback when received msgs
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.vel = Twist()
                # Set a publish rate of in 10 Hz, the time interval will be 0.1s
        self.rate = rospy.Rate(10)
        self.counter = 0
        # store trajectory
        self.trajectory = list()
        # self initial pose
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        # Run the code
        self.run()

    def move_forward(self, distance, speed = 0.1):
        self.vel.linear.x = speed
        self.vel.angular.z = 0
        time_to_move = distance / speed
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < time_to_move:
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.vel.linear.x = 0
        self.vel_pub.publish(self.vel)

    def rotate(self, angle, speed=0.2):
        self.vel.linear.x = 0
        self.vel.angular.z = speed
        time_to_turn = angle / speed
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < time_to_turn:
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        # Update the pose to include (x, y, theta)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
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
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)
        
    def run(self):
        for _ in range(4): # 4 loops
            self.move_forward(1)
            rospy.sleep(0.2)
            self.rotate(pi/2)
            rospy.sleep(0.2)
        self.stop()
        self.plot_trajectory()

if __name__ == '__main__':
    try:
        TurtlebotDriving()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminated")
