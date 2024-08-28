#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveForward:
    def __init__(self):
        rospy.init_node('move_forward_node', anonymous=True)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.distance_traveled = 0.0
        self.start_position = None
        self.is_moving = False

        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.on_shutdown(self.stop_turtlebot)

    def odom_callback(self, msg):
        position = msg.pose.pose.position

        if self.start_position is None:
            self.start_position = position

        self.distance_traveled = self.calculate_distance(self.start_position, position)

        if self.distance_traveled >= 1.0:
            self.stop_turtlebot()
            rospy.loginfo("1 metre ileri gidildi.")
            self.is_moving = False

    def calculate_distance(self, start, current):
        return ((current.x - start.x) ** 2 + (current.y - start.y) ** 2) ** 0.5

    def stop_turtlebot(self):
        rospy.loginfo("TurtleBot'u durdur.")
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)

    def move_forward(self):
        rospy.loginfo("TurtleBot 1 metre ileri gidiyor.")
        self.is_moving = True
        vel_msg = Twist()
        vel_msg.linear.x = 0.2 

        while not rospy.is_shutdown() and self.is_moving:
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        bot = MoveForward()
        bot.move_forward()
    except rospy.ROSInterruptException:
        pass