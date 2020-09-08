#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String


class Camera_Vizer:
    def __init__(self, color_topic, depth_topic):

        ### Define the subscribers
        self.color_sub = rospy.Subscriber(color_topic, Image, self.color_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
        self.BB_sub = rospy.Subscriber('/tracking_node/tracking_BB', RegionOfInterest, self.BB_callback)
        self.track_status_sub = rospy.Subscriber('/tracking_node/tracking_status', String, self.track_status_callback)

        self.init_tracking_pub = rospy.Publisher('/tracking_node/init_tracking', RegionOfInterest, queue_size=10)

        ### Some general placeholders
        self.img_size_x = 640
        self.img_size_y = 480
        self.color_image = np.zeros((self.img_size_y, self.img_size_x, 3), np.int8)
        self.color_image_drawing = np.copy(self.color_image)

        self.depth_image = np.zeros((self.img_size_x, self.img_size_y, 3), np.int8)
        self.bridge = CvBridge()

        ### The openCV settings
        cv2.namedWindow('image_visualizer', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('image_visualizer', self.mouse_callback)

        ### parameters for the bounding box for initialization
        self.x_width = 75
        self.y_width = 100
        self.BB = (0,0,0,0)

        ### Variables to trakc current state of everything
        self.status = 'initializing'
        # self.tracking_status = 'waiting'


    def mouse_callback(self, event, x, y, flags, param):

        print 'Mouse callback status', self.status

        if self.status == 'initializing':
            if event == cv2.EVENT_MOUSEMOVE:
                self.color_image_drawing = np.copy(self.color_image)

                p1_x = max(0, x - self.x_width / 2)
                p1_y = max(0, y - self.y_width / 2)

                p2_x = min(self.img_size_x - 1, x + self.x_width / 2)
                p2_y = min(self.img_size_y - 1, y + self.y_width / 2)

                cv2.rectangle(self.color_image_drawing, (p1_x, p1_y), (p2_x, p2_y), color = (0, 255, 0), thickness = 1)

            if event == cv2.EVENT_LBUTTONDOWN:

                self.status = 'tracking'
                init_track_msg = RegionOfInterest()
                init_track_msg.x_offset = max(0, x - self.x_width / 2)
                init_track_msg.y_offset = max(0, y - self.y_width / 2)
                init_track_msg.height = self.y_width
                init_track_msg.width = self.x_width
                self.init_tracking_pub.publish(init_track_msg)

    def BB_callback(self, roi_msg):
        start_x = roi_msg.x_offset
        start_y = roi_msg.y_offset
        width = roi_msg.width
        height = roi_msg.height

        self.BB = (start_x, start_y, width, height)

    def track_status_callback(self, ros_string):
        if ros_string.data == 'waiting':
            self.status = 'initializing'
        else:
            self.status = 'tracking'

    def color_callback(self, ros_image):

        # print 'got a new image!'

        self.color_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')

        if self.status == 'initializing':
            self.color_image_drawing = np.copy(self.color_image)
            print 'image classback initializing'
        elif self.status == 'tracking':
            self.color_image_drawing = np.copy(self.color_image)
            print 'image classback tracking'
            if self.BB is not None:
                # print 'new bounding box', self.BB
                start_x, start_y, width, height = self.BB
                cv2.rectangle(self.color_image_drawing, (start_x, start_y), (start_x + width, start_y + height),
                              color = (0, 255, 0), thickness = 1)


        self.img_size_x = self.color_image_drawing.shape[1]
        self.img_size_y = self.color_image_drawing.shape[0]

    def depth_callback(self, ros_depth_image):
        pass

    def show_image(self):

        cv2.imshow('image_visualizer', self.color_image_drawing)
        cv2.waitKey(1)
        # pass


if __name__ == "__main__":

    ## delete this code
    # img = np.ones((480,640,3),np.int8)
    # img4 = cv2.rectangle(img, (20, 20) , (50,50),(0,255,0),-1)
    # cv2.imshow('test img', img)
    # key = cv2.waitKey(5000)

    # if key == ord('s'):
    # initBB = cv2.selectROI("Frame", img, fromCenter=False,
    #                        showCrosshair=True)
    # print initBB

    ##

    rospy.init_node('camera_visualizer')

    camera_color_topic = rospy.get_param('color_image_topic', '/camera/color/image_raw')
    camera_depth_topic = rospy.get_param('depth_image_topic', '/camera/depth/image_rect_raw')

    rate = rospy.Rate(25)

    cs = Camera_Vizer(color_topic=camera_color_topic,
                      depth_topic=camera_depth_topic)


    while not rospy.is_shutdown():

        cs.show_image()
        rate.sleep()

    cv2.destroyAllWindows()

