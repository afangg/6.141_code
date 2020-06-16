#!/usr/bin/python

# Use this file to select proper HSV ranges for color filtering,
# Then modify the constants in cv_utils.py
# The UI interface(slider) should be straightforward

# Adapted from: https://github.com/mit-bwsi-racecar-ms/online-labs/blob/master/utils.py
# To run: python hsv_select_live.py

import numpy as np
import cv2

VIDEO_PORT = 4


def hsv_select_live():
    '''Thresholds live video via HSV Trackbar values.'''
    window_name = 'HSV Select Live'
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    video = cv2.VideoCapture(VIDEO_PORT)
    hsv_min = np.array([0,0,0])
    hsv_max = np.array([179,255,255])
    def update():
        frame = video.read()[1]
        if frame is not None:
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_min[0] = cv2.getTrackbarPos('H_min', window_name)
            hsv_max[0] = cv2.getTrackbarPos('H_max', window_name)
            hsv_min[1] = cv2.getTrackbarPos('S_min', window_name)
            hsv_max[1] = cv2.getTrackbarPos('S_max', window_name)
            hsv_min[2] = cv2.getTrackbarPos('V_min', window_name)
            hsv_max[2] = cv2.getTrackbarPos('V_max', window_name)
            mask = cv2.inRange(img_hsv, hsv_min, hsv_max)
            img_masked = cv2.bitwise_and(frame, frame, mask = mask)
            cv2.putText(img_masked, 'HSV Lower: {}'.format(hsv_min), (10, 35), 0, 0.75, (255, 255, 255), 2)
            cv2.putText(img_masked, 'HSV Upper: {}'.format(hsv_max), (10, 70), 0, 0.75, (255, 255, 255), 2)
            cv2.imshow(window_name, img_masked)
    def callback(value):
        update()
    # make the trackbar used for HSV masking
    cv2.createTrackbar('H_min', window_name, 0, 179, callback)
    cv2.createTrackbar('H_max', window_name, 0, 179, callback)
    cv2.createTrackbar('S_min', window_name, 0, 255, callback)
    cv2.createTrackbar('S_max', window_name, 0, 255, callback)
    cv2.createTrackbar('V_min', window_name, 0, 255, callback)
    cv2.createTrackbar('V_max', window_name, 0, 255, callback)
    # set initial trackbar values
    cv2.setTrackbarPos('H_min', window_name, 0)
    cv2.setTrackbarPos('H_max', window_name, 179)
    cv2.setTrackbarPos('S_min', window_name, 0)
    cv2.setTrackbarPos('S_max', window_name, 255)
    cv2.setTrackbarPos('V_min', window_name, 0)
    cv2.setTrackbarPos('V_max', window_name, 255)
    # wait for 'ESC' destroy windows
    while cv2.waitKey(200) & 0xFF != 27:
        update()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

hsv_select_live()
