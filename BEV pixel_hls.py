#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge

bridge = CvBridge()
current_frame = None

def image_cb(msg):
    global current_frame
    # 원본 해상도(예:640×480)로 바로 디코딩
    current_frame = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

def on_mouse(event, x, y, flags, param):
    global current_frame
    if event == cv2.EVENT_LBUTTONDOWN and current_frame is not None:
        bgr = current_frame[y, x]
        pixel = np.uint8([[bgr]])
        hls = cv2.cvtColor(pixel, cv2.COLOR_BGR2HLS)[0][0]
        hsv = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
        print('픽셀 ({}, {})  BGR={}  HLS={}  HSV={}'
              .format(x, y, bgr.tolist(), hls.tolist(), hsv.tolist()))

def main():
    rospy.init_node('hls_hsv_sampler', anonymous=True)
    rospy.Subscriber('/csi_cam_0/image_raw/compressed',
                     CompressedImage, image_cb, queue_size=1)

    win = 'Live Sampler'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse, None)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if current_frame is not None:
            cv2.imshow(win, current_frame)
            cv2.waitKey(1)
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
