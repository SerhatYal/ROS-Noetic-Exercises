#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from service.srv import CheckDistance, CheckDistanceResponse

class TurtleSimService:
    def __init__(self):
        self.distance_threshold = 5.0
        self.starting_pose = None
        self.current_pose = Pose()

        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        
        self.service = rospy.Service('check_distance', CheckDistance, self.handle_check_distance)

    def pose_callback(self, pose):
        if self.starting_pose is None:
            self.starting_pose = pose
        self.current_pose = pose

    def handle_check_distance(self, req):
        
        if self.starting_pose is None:
            rospy.loginfo("Başlangiç pozisyonu ayarlanmamiş.")
            return CheckDistanceResponse(False)
        
        distance = ((self.starting_pose.x - self.current_pose.x) ** 2 + (self.starting_pose.y - self.current_pose.y) ** 2) ** 0.5
        
        # Mesafe belirlenen eşiği aştıysa mesajı yayınla
        distance_exceeded = distance >= req.target_distance
        if distance_exceeded:
            rospy.loginfo("Asansör kaldirildi!")
        return CheckDistanceResponse(distance_exceeded)

if __name__ == "__main__":
    rospy.init_node('turtle_sim_service_node')
    service = TurtleSimService()
    rospy.loginfo("Mesafe kontrol servisi çalişiyor...")

    rospy.spin()