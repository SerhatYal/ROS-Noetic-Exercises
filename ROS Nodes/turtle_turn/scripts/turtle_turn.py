#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def reset_turtle():
    rospy.wait_for_service('/reset')
    try:
        reset_turtle = rospy.ServiceProxy('/reset', Empty)
        reset_turtle()
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def draw_circle(pub, radius):
    move_cmd = Twist()
    move_cmd.linear.x = radius
    move_cmd.angular.z = 1.0  
    for _ in range(56):
        pub.publish(move_cmd)
        rospy.sleep(0.1)  

def draw_square(pub, side_length):
    move_cmd = Twist()
    for _ in range(4):
        move_cmd.linear.x = side_length  
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1.0)  

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.57  
        pub.publish(move_cmd)
        rospy.sleep(1.0)

        print_pose_info()

def draw_triangle(pub, side_length):
    move_cmd = Twist()
    for _ in range(3):
        move_cmd.linear.x = side_length  
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.sleep(1.0)  

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 2.09  
        pub.publish(move_cmd)
        rospy.sleep(1.0)

        print_pose_info()

def print_pose_info():
    rospy.loginfo(f"Current Pose - X: {current_pose.x}, Y: {current_pose.y}, Theta: {current_pose.theta}")

def main():
    rospy.init_node('turtle_shape_drawer')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    shape = input("Çizmek istediğiniz şekli girin (daire/kare/üçgen): ").strip().lower()

    if shape == "daire":
        radius = float(input("Dairenin çapini girin (örneğin 2.0): "))
        rospy.set_param('/turtlesim/background_r', 0)
        rospy.set_param('/turtlesim/background_g', 255)
        rospy.set_param('/turtlesim/background_b', 0)
        reset_turtle()
        draw_circle(pub, radius / 2)  # Çap yerine yarıçap kullanarak çiziyoruz
    elif shape == "kare":
        side_length = float(input("Karenin kenar uzunluğunu girin (örneğin 2.0): "))
        rospy.set_param('/turtlesim/background_r', 255)
        rospy.set_param('/turtlesim/background_g', 255)
        rospy.set_param('/turtlesim/background_b', 0)
        reset_turtle()
        rospy.sleep(1.0)
        draw_square(pub, side_length)

    elif shape == "üçgen":
        side_length = float(input("Üçgenin kenar uzunluğunu girin (örneğin 2.0): "))
        rospy.set_param('/turtlesim/background_r', 255)
        rospy.set_param('/turtlesim/background_g', 0)
        rospy.set_param('/turtlesim/background_b', 0)
        reset_turtle()
        rospy.sleep(1.0)
        draw_triangle(pub, side_length)

    else:
        print("Geçersiz şekil girdiniz.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass