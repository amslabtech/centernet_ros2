#!/usr/bin/env python
#! coding: utf-8

import os
import sys
import cv2
import time
import argparse
import yaml
import json
from pprint import pprint

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## from https://github.com/xingyizhou/CenterNet/blob/master/src/lib/utils/debugger.py
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
    def __init__(self):
        super().__init__('object_detector')
        # load config file
        config_file_path = os.path.dirname(__file__) + '/config/config.yaml'
        print('load config from' + config_file_path)
        config = dict()
        with open(config_file_path) as f:
            config = yaml.load(f)
        pprint(config)

        self.image_sub = self.create_subscription(Image, config['input_image_topic'], self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, config['detection_image_topic'], 1)
        self.bbox_pub = self.create_publisher(String, config['bounding_boxes_topic'], 1)

        self.bridge = CvBridge()

        self.CONFIDENCE_THRESHOLD = config['confidence_threshold']

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
        print('INPUT_IMAGE_TOPIC:', config['input_image_topic'])
        print('DETECTION_IMAGE_TOPIC:', config['detection_image_topic'])
        print('BOUNDING_BOXES_TOPIC:', config['bounding_boxes_topic'])
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
                    confidence = obj.tolist()[4]
                    if confidence > self.CONFIDENCE_THRESHOLD:
                        bbox = obj.tolist()[:4]
                        # bbox: [umin, vmin, umax, vmax]
                        category = i - 1
                        text = coco_class_name[category] + ', {:.3f}'.format(confidence)
                        print(text)
                        bounding_box = {"class": coco_class_name[category], "confidence": confidence, "xmin": bbox[0], "ymin": bbox[1], "xmax": bbox[2], "ymax": bbox[3]}
                        bounding_boxes['list'].append(bounding_box)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        text_size = cv2.getTextSize(text, font, 0.5, 2)[0]
                        text_box = [int(bbox[0]), int(bbox[1] - text_size[1]), int(bbox[0] + text_size[0]), int(bbox[1])]
                        cv2.rectangle(image, (text_box[0], text_box[1]), (text_box[2], text_box[3]), (255, 255, 0), -1)
                        cv2.putText(image, text, (int(bbox[0]), int(bbox[1] - 2)), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 255, 0), 2)
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

def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
