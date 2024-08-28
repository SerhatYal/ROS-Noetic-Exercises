#!/usr/bin/env python3
import rospy
import time
from std_srvs.srv import Empty
from std_msgs.msg import String

def reset_turtle():
    rospy.wait_for_service('/reset')
    try:
        reset_turtle = rospy.ServiceProxy('/reset', Empty)
        reset_turtle()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def talker():
    # ROS node'u başlat
    rospy.init_node('talker_listener', anonymous=True)

    # 'chatter' topic'inde String türünde veri yayınlayacak bir Publisher oluştur
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Yayın hızını belirle
    rate = rospy.Rate(10)  # 10 Hz
    
    t = 40

    while not rospy.is_shutdown():
        # Gönderilecek mesajı oluştur
        msg = String()
        time.sleep(0.25)
        t += 1
        msg.data = str(t)  # Mesaj verisini buraya yazın

        # Arka plan rengini güncelle
        if t <= 50:
            rospy.set_param('/turtlesim/background_r', 0)
            rospy.set_param('/turtlesim/background_g', 255)
            rospy.set_param('/turtlesim/background_b', 0)
            reset_turtle()
        elif t > 50 and t <= 75:
            rospy.set_param('/turtlesim/background_r', 255)
            rospy.set_param('/turtlesim/background_g', 255)
            rospy.set_param('/turtlesim/background_b', 0)
            reset_turtle()
        elif t > 75 and t <= 100:
            rospy.set_param('/turtlesim/background_r', 255)
            rospy.set_param('/turtlesim/background_g', 0)
            rospy.set_param('/turtlesim/background_b', 0)
            reset_turtle()

        # Mesajı yayınla
        rospy.loginfo("Publishing: %s", msg.data)
        pub.publish(msg)

        

        # `t` değeri 100'ü geçerse döngüden çık
        if t >= 100:
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
