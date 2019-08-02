#!/usr/bin/env python
#! coding: utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/detected_image', 1)

    def image_callback(self, msg):
        print('callback')

def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
