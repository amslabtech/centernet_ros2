#!/usr/bin/env python
#! coding: utf-8

import os
import sys
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/detection_image', 1)
        self.bridge = CvBridge()

        # CenterNet
        CENTERNET_PATH = '/root/CenterNet/src'
        sys.path.append(CENTERNET_PATH)
        import _init_paths
        from detectors.detector_factory import detector_factory
        from opts import opts
        MODEL_PATH = CENTERNET_PATH + '/../models/ctdet_coco_dla_2x.pth'
        TASK = 'ctdet'
        opt = opts().init('{} --load_model {}'.format(TASK, MODEL_PATH).split(' '))

        self.detector = detector_factory[opt.task](opt)

    def image_callback(self, msg):
        print('callback')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print(cv_image)
            cv2.imshow('image', cv_image)
            cv2.waitkey(0)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
