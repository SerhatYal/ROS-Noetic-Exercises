#!/usr/bin/env python3

import threading
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Log

class TurtleBotNavigator:

    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # İlk pozisyonu alma
        self.starting_pose = self.get_current_pose()

        # Önceden belirlenmiş üç hedef noktası
        self.goals = [
        
            Pose(Point(-1.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(7.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(1.0, 3.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            self.starting_pose
        ]

        self.current_goal_index = 0
        self.navigation_thread = None
        self.is_navigating = False
        self.last_goal_index = -1
        self.goal_reached = False

        # Topic'leri advertise et
        self.start_pub = rospy.Publisher('/start_robot', String, queue_size=10)
        self.stop_pub = rospy.Publisher('/stop_robot', String, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)  # ROS Bridge için yeni publisher
        self.log_pub = rospy.Publisher('/ros_log_messages', String, queue_size=10) # ROS logları için publisher

        # ROS Bridge üzerinden komutları dinlemek için subscriber ekle
        rospy.Subscriber("/start_robot", String, self.ros_command_callback)
        rospy.Subscriber("/stop_robot", String, self.ros_command_callback)

        # Log mesajları için subscriber ekle
        rospy.Subscriber('/rosout_agg', Log, self.log_callback)

    def log_callback(self, data):
        # Sadece navigation_node_2.py tarafından üretilen logları yayınla
        if data.name == rospy.get_name():
            self.log_pub.publish(data.msg)

    def ros_command_callback(self, msg):
        if msg.data == "start":
            self.handle_start()
        elif msg.data == "stop":
            self.handle_stop()

    def get_current_pose(self):
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
        self.client.wait_for_result()

        # Hedef tamamlandı mı kontrol et
        result = self.client.get_state()
        return result == actionlib.GoalStatus.SUCCEEDED

    def start_navigation(self):
        self.status_pub.publish("Starting navigation...")  # Status mesajı gönder
        rospy.loginfo("Starting navigation...")
        while self.current_goal_index < len(self.goals):
            if not self.is_navigating:
                break
            goal = self.goals[self.current_goal_index]
            result = self.move_to_goal(goal)
            if result:
                message = f"Reached goal {self.current_goal_index + 1} successfully!"
                self.status_pub.publish(message)  # Status mesajı gönder
                rospy.loginfo(message)
                self.last_goal_index = self.current_goal_index
                self.current_goal_index += 1
                self.goal_reached = True
            else:
                
                self.goal_reached = False

        self.is_navigating = False

    def handle_start(self):
        if not self.is_navigating:
            self.is_navigating = True
            if not self.goal_reached and self.current_goal_index == self.last_goal_index:
                self.status_pub.publish(f"Resuming goal {self.current_goal_index + 1}")  # Status mesajı gönder
                rospy.loginfo(f"Resuming goal {self.current_goal_index + 1}")
                self.navigation_thread = threading.Thread(target=self.start_navigation)
                self.navigation_thread.start()
            else:
                
                self.navigation_thread = threading.Thread(target=self.start_navigation)
                self.navigation_thread.start()
        else:
            rospy.loginfo("Already navigating...")

    def handle_stop(self):
        if self.is_navigating:
            self.is_navigating = False
            self.client.cancel_goal()  # Şu anki hedefi iptal eder
            if self.navigation_thread:
                self.navigation_thread.join()
            self.status_pub.publish("Navigation stopped.")  # Status mesajı gönder
            rospy.loginfo("Navigation stopped.")
            self.goal_reached = False
        else:
            rospy.loginfo("Not currently navigating.")

if __name__ == '__main__':
    try:
        navigator = TurtleBotNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
