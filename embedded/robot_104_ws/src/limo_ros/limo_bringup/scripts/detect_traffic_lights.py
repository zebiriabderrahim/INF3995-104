#!/usr/bin/env python
import cv2
import numpy as np
import random
import time
from enum import Enum

from read_realsense_image import ReadImage

import rospy
from std_msgs.msg import Int16

pub = None
ri = ReadImage()

def DetectColor(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_min = np.array([0, 5, 150])
    red_max = np.array([8, 255, 255])
    red_min2 = np.array([175, 5, 150])
    red_max2 = np.array([180, 255, 255])

    yellow_min = np.array([20, 5, 150])
    yellow_max = np.array([30, 255, 255])

    green_min = np.array([35, 5, 150])
    green_max = np.array([90, 255, 255])

    red_thresh = cv2.inRange(hsv_img, red_min, red_max) + \
        cv2.inRange(hsv_img, red_min2, red_max2)
    yellow_thresh = cv2.inRange(hsv_img, yellow_min, yellow_max)
    green_thresh = cv2.inRange(hsv_img, green_min, green_max)

    red_blur = cv2.medianBlur(red_thresh, 5)
    yellow_blur = cv2.medianBlur(yellow_thresh, 5)
    green_blur = cv2.medianBlur(green_thresh, 5)

    red = cv2.countNonZero(red_blur)
    yellow = cv2.countNonZero(yellow_blur)
    green = cv2.countNonZero(green_blur)

    light_color = max(red, yellow, green)

    if light_color > 60:
        if light_color == red:
            return 1
        elif light_color == yellow:
            return 2
        elif light_color == green:
            return 3
    else:
        return 0


class TLState(Enum):  # traffic light state
    red = 1
    yellow = 2
    green = 3


class TLType(Enum):  # traffic light type
    regular = 0
    five_lights = 1
    four_lights = 2


def ImageResize(image, height, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    r = height / float(h)
    dim = (int(w*r), height)
    resized = cv2.resize(image, dim, interpolation=inter)
    return resized


def DetectState(image, type):
    image = ImageResize(image, 200)
    cimage = image.copy()
    (height, width) = image.shape[:2]
    output = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=30)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        a, b, c = circles.shape
        for i in range(b):
            cv2.circle(cimage, (circles[0][i][0], circles[0][i][1]),
                       circles[0][i][2], (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(cimage, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3,
                       cv2.LINE_AA)  # draw center of circle

    cv2.imshow("circle_img", cimage)
    cv2.waitKey(2000)

    overallState = 0
    stateArrow = 0
    stateSolid = 0

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            if i[1] < i[2]:
                i[1] = i[2]

            roi = image[(i[1]-i[2]):(i[1]+i[2]), (i[0]-i[2]):(i[0]+i[2])]

            color = DetectColor(roi)
            if color > 0:
                stateSolid = color

        if type == 1:
            overallState = stateArrow + stateSolid + 1
        elif type == 2:
            overallState = stateArrow + 7
        else:
            overallState = stateSolid

    return overallState


def PlotLightResult(images):
    for i, image_path in enumerate(images):
        image = cv2.imread(image_path)
        image_roi = image[0:260, 400:640]

        state = DetectState(image_roi, TLType.regular.value)
        if state > 0:
            label = TLState(state).name
            print("detect result: " + label)

        cv2.imshow("image_roi", image_roi)
        cv2.waitKey(2000)


def PlotRealsenseResult(image):
    image_roi = image[0:260, 400:640]

    state = DetectState(image_roi, TLType.regular.value)
    if state > 0:
        label = TLState(state).name
        print("detect result: " + label)
    else:
        print("can not detect image result")
    cv2.imshow("image_roi", image_roi)
    cv2.waitKey(2000)

    return state


def callback(data):

    if data.data != -1:
        return None

    rospy.loginfo(rospy.get_caller_id() +
        "I get traffic light: %s", data.data)


    image = ri.read()
    if image is not None:
        color_idx = PlotRealsenseResult(image)
    else:
        return None

    if color_idx == 1:
        pub.publish(Int16(0))
    elif color_idx == 3:  # 0 red, 1 green
        pub.publish(Int16(1))
    else:
        print("can not detect realsense image result")

    time.sleep(1)


def ROSSubscribe():
    rospy.init_node('traffic_light_detecter')
    rospy.Subscriber("traffic_light_status", Int16, callback)

    global pub
    pub = rospy.Publisher("traffic_light_status", Int16, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":

    # #Test 1.  use local image
    # light_img_path = ["images/red.jpg", "images/yellow.png", "images/green.png"]
    # light_img_path = ["traffic_light_red_roi.png", "traffic_light_yellow_roi.png", "traffic_light_green_roi.png"]
    # light_img_path = ["red_Color.png"]
    # random.shuffle(light_img_path)
    # PlotLightResult(light_img_path)

    # #Test 2.  use realsense capture image
    ri = ReadImage()
    image = ri.read()
    if image is not None:
        PlotRealsenseResult(image)
    else:
        print("can not read realsense image")

    # ROSSubscribe()
