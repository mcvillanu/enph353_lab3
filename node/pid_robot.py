#! /usr/bin/env python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(100)
move = Twist()

bridge = CvBridge()

lastPosition = 400

def returnPosition(thresh):
    global lastPosition
    xSum = 0
    xCount = 0
    lookFirst = True
    lookSecond = False

    for i in range(765,795):
        for j in range(1, 800):
           # print("row:", i, "column:",j, "value:", thresh[i][j])
            if(lookFirst and thresh[i][j] < 127):
                xSum += j
                xCount += 1
                lookFirst = False
                lookSecond = True
            if(lookSecond and thresh[i][j] > 127):
                xSum += j
                xCount += 1
                lookFirst = True
                lookSecond = False

    if xCount > 30:
        global lastPosition
        lastPosition = (int)(xSum/xCount)
        return lastPosition
    else:
        return lastPosition

def imageCallback(data):
    global lastPosition
    print("Recieved an image!")

    try:
        kernel = np.ones((10,10),np.uint)

        rgb_image = bridge.imgmsg_to_cv2(data)
        gray_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray_frame,90,255, cv2.THRESH_BINARY)

        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        s_error = 400 - returnPosition(opening)

        print(s_error)

        if(abs(s_error) > 70):
            move.linear.x = 0
            move.angular.z = 0.4 * s_error/abs(s_error)
        else:
            move.linear.x = 0.15
            move.angular.z = 0

        pub.publish(move)
    except CvBridgeError, e:
        print(e)

while not rospy.is_shutdown():
    image_topic = "/camera1/image_raw"
    sub_cam = rospy.Subscriber(image_topic, Image, imageCallback)
    rospy.spin()

