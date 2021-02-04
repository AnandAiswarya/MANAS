#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from lane_detection.msg import LaneCoords

def run(lsa, lca, rsa, rca):
    pub = rospy.Publisher('coords_topic', LaneCoords, queue_size=10)

    test = LaneCoords()
    test.header.stamp = rospy.Time.now()
    test.LeftCoeff = [lsa,lca]
    test.RightCoeff = [rsa, rca]
    pub.publish(test)



leftLineCoordinate = (300, 750)
rightLineCoordinate = (1100, 750)


leftSlopeValues, leftConstantValues, rightSlopeValues, rightConstantValues = [], [], [], [];


def calculateDistance(first, second):
    tempFirst = np.square(first[0] - second[0])
    tempSecond = np.square(first[1] - second[1])
    return np.sqrt(tempFirst + tempSecond)


def canny_edge_detector(image):
    # Convert the image color to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Reduce noise from the image
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image):
    height = image.shape[0]
    polygons = np.array([
        [(100, height), (1300, height), (650, 450)]
    ])
    mask = np.zeros_like(image)

    # Fill poly-function deals with multiple polygon
    cv2.fillPoly(mask, polygons, 255)

    # Bitwise operation between canny image and mask image
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def draw_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    leftSlopeValues, rightSlopeValues, leftConstantValues, rightConstantValues = [], [], [], []
    h = img.shape[0]

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                leftDistance1 = calculateDistance((x1, y1), leftLineCoordinate)
                rightDistance1 = calculateDistance((x1, y1), rightLineCoordinate)


                if leftDistance1 < rightDistance1:

                    slopeLeft = -(float(y2 - y1) / float(x2 - x1))
                    constLeft = float((h-y2)) - float((slopeLeft * x2))
                    leftSlopeValues.append(slopeLeft)
                    leftConstantValues.append(constLeft)


                else:

                    slopeRight = float((y1 - y2)) / float((x2 - x1))
                    constRight = float(h-y2) - float(slopeRight * x2)
                    rightSlopeValues.append(slopeRight)
                    rightConstantValues.append(constRight)

                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        leftSlopeAverage = np.average(leftSlopeValues)
        leftConstantAverage = np.average(leftConstantValues)
        rightSlopeAverage = np.average(rightSlopeValues)
        rightConstantAverage = np.average(rightConstantValues)
        leftSlopeValues = []
        leftConstantValues = []
        rightSlopeValues = []
        rightConstantValues = []

        run(leftSlopeAverage,leftConstantAverage,rightConstantAverage,rightConstantAverage)
        # print('LEFT SLOPE = ' + str(leftSlopeAverage))
        # print('LEFT CONST= ' + str(leftConstantAverage))
        # print('RIGHT SLOPE= ' + str(rightSlopeAverage))
        # print('RIGHT CONST = ' + str(rightConstantAverage))



    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)

    return img


def callback(data):
    # convert sensor_msgs/Image to OpenCV Image
    
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    # white color mask
    lower = np.uint8([0, 200, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(hls, lower, upper)
    # yellow color mask
    lower = np.uint8([10, 0, 100])
    upper = np.uint8([40, 255, 255])
    yellow_mask = cv2.inRange(hls, lower, upper)
    # combine the mask

    combo_mask = cv2.bitwise_or(white_mask, yellow_mask)

    final_mask_combo = cv2.bitwise_and(frame, frame, mask=combo_mask)
    cropped_image = region_of_interest(final_mask_combo)

    canny_image = canny_edge_detector(cropped_image)
    combined_lines = cv2.HoughLinesP(canny_image, rho=2, theta=np.pi / 180,
                                     threshold=50, lines=np.array([]), minLineLength=20, maxLineGap=70)

    image_with_lines = draw_lines(frame, combined_lines)




def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("imageTopicTest", Image, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()