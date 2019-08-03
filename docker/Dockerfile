FROM nvidia/cuda:9.2-cudnn7-devel-ubuntu18.04

SHELL ["/bin/bash", "-c"]

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -q -y \
        curl \
        gnupg2 \
        lsb-release \
        wget \
        libsm6 \
        libxext6 \
        libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl http://repo.ros2.org/repos.key | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install --no-install-recommends -y \
        git \
        python3-pip \
        python3-rospkg \
        python3-catkin-pkg \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U \
        argcomplete \
        setuptools \
        http://download.pytorch.org/whl/cu92/torch-0.4.1-cp36-cp36m-linux_x86_64.whl \
        torchvision==0.2.2 \
        cython \
        cffi

RUN apt-get update && apt-get install -y \
        ros-dashing-ros-base \
        ros-dashing-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/dashing/setup.bash" >> /root/.bashrc

WORKDIR /root

RUN mkdir -p ros2_ws/src

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"

RUN echo 'export PATH=/usr/local/cuda/bin:${PATH}' >> ~/.bashrc
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc

# set default python vertion 3
RUN rm -f /usr/bin/python
RUN ln -s /usr/bin/python3 /usr/bin/python

RUN sed -i "1254s/torch\.backends\.cudnn\.enabled/False/g" /usr/local/lib/python3.6/dist-packages/torch/nn/functional.py

# build centernet
RUN git clone https://github.com/cocodataset/cocoapi.git --depth=1 /root/cocoapi

WORKDIR /root/cocoapi/PythonAPI

RUN make && python setup.py install

RUN git clone https://github.com/xingyizhou/CenterNet.git --depth=1 /root/CenterNet

WORKDIR /root/CenterNet

RUN pip3 install -r requirements.txt

WORKDIR /root

RUN wget "https://drive.google.com/uc?export=download&id=1pl_-ael8wERdUREEnaIfqOV_VF2bEVRT" -O ctdet_coco_dla_2x.pth && mv ctdet_coco_dla_2x.pth /root/CenterNet/models

WORKDIR /root

# WORKDIR /root/centernet/src/lib/models/networks/DCNv2
#
# RUN ./make.sh

COPY ./ros_entrypoint.sh /
CMD ["/ros_entrypoint.sh"]
