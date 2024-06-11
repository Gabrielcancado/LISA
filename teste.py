#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def distance_callback(data):
    rospy.loginfo(f"Distância do rosto até a câmera: {data.data:.2f} cm")

def distance_subscriber():
    rospy.init_node('distance_subscriber_node', anonymous=True)
    rospy.Subscriber('/face_distance', Float32, distance_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        distance_subscriber()
    except rospy.ROSInterruptException:
        pass
