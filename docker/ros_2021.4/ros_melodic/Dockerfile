# ros openvino toolkit env master ec5b9b9716e4

# image base on ros-melodic
from osrf/ros:melodic-desktop-full

# setting proxy env --option 
ENV http_proxy=http://child-prc.intel.com:913
ENV https_proxy=http://child-prc.intel.com:913

# maintainer information
LABEL maintainer="Pengqiang Li <pengqiang.li@intel.com>"

# default shell type
SHELL ["/bin/bash", "-c"]

# ignore the warning
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --assume-yes apt-utils

#update cmake
WORKDIR /tmp
RUN apt-get install -y wget && wget https://github.com/Kitware/CMake/releases/download/v3.14.4/cmake-3.14.4-Linux-x86_64.sh \
&& chmod +x cmake-3.14.4-Linux-x86_64.sh \
&& ./cmake-3.14.4-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir --skip-license \
&& rm ./cmake-3.14.4-Linux-x86_64.sh

# install openvino 2021.4
# https://docs.openvinotoolkit.org/latest/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install curl gnupg2 lsb-release
RUN curl -s https://apt.repos.intel.com/openvino/2021/GPG-PUB-KEY-INTEL-OPENVINO-2021 |apt-key add -
RUN echo "deb https://apt.repos.intel.com/openvino/2021 all main" | tee /etc/apt/sources.list.d/intel-openvino-2021.list
RUN apt update
RUN apt-cache search openvino
RUN apt-get install -y intel-openvino-dev-ubuntu20-2021.4.582 
RUN ls -lh /opt/intel/openvino_2021
RUN source /opt/intel/openvino_2021/bin/setupvars.sh 

# install librealsense2
RUN apt-get install -y --no-install-recommends \
software-properties-common 
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-key adv --keyserver-options http-proxy=http://child-prc.intel.com:913/ --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=http://child-prc.intel.com:913/ --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u \
&& apt-get install -y --no-install-recommends \
librealsense2-dkms \
librealsense2-utils \
librealsense2-dev \
librealsense2-dbg \
libgflags-dev \
libboost-all-dev \
&& rm -rf /var/lib/apt/lists/*

# other dependencies
RUN apt-get update && apt-get install -y python3-pip && python3 -m pip install -U \
numpy \
networkx \
pyyaml \
requests \
&& apt-get install -y --no-install-recommends libboost-all-dev
WORKDIR /usr/lib/x86_64-linux-gnu
RUN ln -sf libboost_python-py36.so libboost_python37.so


# build ros openvino toolkit
# set env before build ros openvino tolkkit
WORKDIR /root
RUN mkdir -p ros_ws/src
WORKDIR /root/ros_ws/src
RUN git init && git clone -b dev-ov2021.4 https://github.com/pqLee/ros_openvino_toolkit.git \
&& git clone https://github.com/intel/object_msgs.git \
&& git clone -b melodic https://github.com/ros-perception/vision_opencv.git \
&& git clone https://github.com/intel-ros/realsense.git
WORKDIR /root/ros_ws/src/realsense
RUN git checkout 2.1.3
WORKDIR /root/ros_ws/
RUN source /opt/ros/melodic/setup.bash && source /opt/intel/openvino_2021/bin/setupvars.sh && catkin_make_isolated
