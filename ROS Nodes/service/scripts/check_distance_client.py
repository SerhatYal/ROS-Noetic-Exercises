#!/usr/bin/env python3

import rospy
from service.srv import CheckDistance

def check_distance_client(target_distance):
    rospy.wait_for_service('check_distance')
    try:
        check_distance = rospy.ServiceProxy('check_distance', CheckDistance)
        response = check_distance(target_distance)
        return response.distance_exceeded
    except rospy.ServiceException as e:
        rospy.logerr("Servis çağrisi başarisiz oldu: %s" % e)

if __name__ == "__main__":
    rospy.init_node('check_distance_client')
    target_distance = 5.0 
    rospy.loginfo("Mesafe kontrol ediliyor...")
    if check_distance_client(target_distance):
        rospy.loginfo(f"Belirlenen {target_distance} metre mesafe aşildi!")
    else:
        rospy.loginfo(f"Belirlenen {target_distance} metre mesafe henüz aşilmadi.")