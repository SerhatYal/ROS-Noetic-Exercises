#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry


class TurtleBotNavigator:

    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # İlk pozisyonu alma
        self.starting_pose = self.get_current_pose()

        # Önceden belirlenmiş üç hedef noktası
        self.goals = [
            self.starting_pose,
            Pose(Point(-1.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(7.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(1.0, 3.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            self.starting_pose
        ]

    def get_current_pose(self):
        # Robotun mevcut pozisyonunu amcl_pose veya odom'dan alıyoruz
        try:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=10)
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            return Pose(Point(position.x, position.y, position.z), Quaternion(orientation.x, orientation.y, orientation.z, orientation.w))
        except:
            rospy.logwarn("Could not get current position. Using default starting pose.")
            return Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    def move_to_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = goal_pose

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def start_navigation(self):
        while not rospy.is_shutdown():
            for goal in self.goals:
                result = self.move_to_goal(goal)
                if result:
                    rospy.loginfo("Reached the goal successfully!")
                else:
                    rospy.loginfo("Failed to reach the goal. Retrying...")

if __name__ == '__main__':
    try:
        navigator = TurtleBotNavigator()
        navigator.start_navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")