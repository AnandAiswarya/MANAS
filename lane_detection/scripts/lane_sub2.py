#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from lane_detection.msg import LaneCoords

def callback(data) :
    print(data.LeftCoeff)
    print(data.RightCoeff)

def listener():
    rospy.init_node('observer', anonymous=True)

    rospy.Subscriber("coords_topic", LaneCoords, callback)

    rospy.spin()




if __name__ == '__main__':
    listener()