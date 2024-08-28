#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.twist = Twist()
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        
        front_distance = min(min(data.ranges[0:30]), min(data.ranges[-30:]))

    
        threshold_distance = 0.3

        if front_distance < threshold_distance:
            
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass