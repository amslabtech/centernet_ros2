# centernet_ros2
[![Build Status](https://travis-ci.org/amslabtech/centernet_ros2.svg?branch=master)](https://travis-ci.org/amslabtech/centernet_ros2)

ROS2 object detector using CenterNet

## Environment and Requirement
### common
- nvidia-driver >= 396
### with docker(recommended)
- Ubuntu 16.04 or 18.04
- docker
- nvidia-docker2
### without docker
- Ubuntu 18.04
- ROS2 (tested on dashing)
- See [CenterNet](https://github.com/xingyizhou/CenterNet) for other requirement

## Dependencies
- [CenterNet](https://github.com/xingyizhou/CenterNet)

## Node
### object_detector
- Published topic
  - /detection_image (sensor_msgs/Image)
    - image with bounding boxes and class name
  - /bounding_boxes (std_msgs/String)
    - json string including bounding_boxes information 
    - sample -> "{'list': [{'class': 'person', 'confidence': '0.8', 'xmin': '270.3', 'ymin': '12.6', 'xmax': '311.9', 'ymax': '39.0'}, {'class': ...}, ...]}"
- Subscribed topic
  - /usb_cam/image_raw (sensor_msgs/Image)

## Install and Build
```
cd your_workspace
git clone https://github.com/amslabtech/centernet_ros2.git
cd centernet_ros2
./build.sh
```

## How to use
```
cd your_workspace/centernet_ros2
./run_docker.sh
(in docker container)
cd /root/ros2_ws/src/centernet_ros2/docker
./make_in_docker.sh
cd /root/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
ros2 run centernet_ros2 object_detector
```

## References
- https://github.com/xingyizhou/CenterNet
