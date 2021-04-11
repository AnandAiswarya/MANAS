#!/usr/bin/env python


import cv2
import numpy
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospkg


def run():
    pub = rospy.Publisher('imageTopicTest', Image, queue_size=10)

    rospy.init_node('talker', anonymous=True)

    rospack = rospkg.RosPack()
    image_path = rospack.get_path('lane_detection') + '/scripts/'
    cap = cv2.VideoCapture(image_path+'challenge_video.mp4')
    if(cap.isOpened()==False):
        print("Error")
    while(cap.isOpened()):
        ret, frame = cap.read()


        bridge = CvBridge()

        imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(imgMsg)
        print("Published!")


    cap.release()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass