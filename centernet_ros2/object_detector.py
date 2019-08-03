#!/usr/bin/env python
#! coding: utf-8

import os
import sys
import cv2
import time
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## from https://github.com/xingyizhou/CenterNet/blob/master/src/lib/utils/debugger.pyjj:w
coco_class_name = [
     'person', 'bicycle', 'car', 'motorcycle', 'airplane',
     'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
     'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
     'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
     'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis',
     'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
     'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass',
     'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich',
     'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
     'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
     'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
     'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
     'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]
## end from

class ObjectDetector(Node):
    def __init__(self, args):
        super().__init__('object_detector')
        self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/detection_image', 1)
        self.bbox_pub = self.create_publisher(String, '/bounding_boxes', 1)
        self.bridge = CvBridge()

        self.CONFIDENCE_THRESHOLD = float(args.confidence_threshold)

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
        print('=== object detector ===')
        print('CONFIDENCE_THRESHOLD:', self.CONFIDENCE_THRESHOLD)
        print('waiting for image...')

    def image_callback(self, msg):
        print('callback')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            start = time.time()
            results = self.detector.run(cv_image)['results']
            inferece_time = time.time() - start
            image = cv_image
            class_num = len(results)
            bounding_boxes = dict()
            bounding_boxes['list'] = list()
            count = 0
            for i in range(1, class_num + 1):
                for obj in results[i]:
                    confidence = obj[4]
                    if confidence > self.CONFIDENCE_THRESHOLD:
                        # print('class {}'.format(i))
                        # print(obj)
                        bbox = obj[:4]
                        # bbox: [umin, vmin, umax, vmax]
                        category = i - 1
                        text = coco_class_name[category]
                        print(text, confidence)
                        bounding_box = {"class": text, "confidence": str(confidence), "xmin": str(bbox[0]), "ymin": str(bbox[1]), "xmax": str(bbox[2]), "ymax": str(bbox[3])}
                        bounding_boxes['list'].append(bounding_box)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        text_size = cv2.getTextSize(text, font, 0.5, 2)[0]
                        text_box = [bbox[0], int(bbox[1] - text_size[1]), int(bbox[0] + text_size[0]), bbox[1]]
                        cv2.rectangle(image, (text_box[0], text_box[1]), (text_box[2], text_box[3]), (255, 255, 0), -1)
                        cv2.putText(image, text, (bbox[0], int(bbox[1] - 2)), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 255, 0), 2)
                        count += 1

            # cv2.namedWindow('image')
            # cv2.imshow('image', image)
            # cv2.waitKey(1)
            # print(bounding_boxes)
            json_string = json.dumps(bounding_boxes)
            # json_dict = json.loads(json_string)
            # print(json_dict)
            bbox_string = String()
            bbox_string.data = json_string
            self.bbox_pub.publish(bbox_string)

            print('inference time: {:.4f}[s]'.format(inferece_time))
            print('fps: {:.4f}'.format(1.0 / inferece_time))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            print('published image')
        except CvBridgeError as e:
            print(e)

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='object_detector')
    parser.add_argument('--confidence-threshold', default=0.3, help='threshold for bounding box adoption')
    parser.add_argument('argv', nargs=argparse.REMAINDER, help='Pass arbitrary arguments to the excutable')

    args = parser.parse_args(argv)
    rclpy.init(args=args.argv)

    node = ObjectDetector(args)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
