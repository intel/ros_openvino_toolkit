# ros openvino toolkit env master ec5b9b9716e4

ARG ROS_VERSION
FROM osrf/ros:${ROS_VERSION}
ARG VERSION

# setting proxy env --option 
# If needed, enable the below ENV setting by correct proxies.
# ENV HTTP_PROXY="your_proxy"
# ENV HTTPS_PROXY="your_proxy"
# ENV FTP_PROXY="your_proxy"

# maintainer information
LABEL maintainer="Jiawei Wu <jiawei.wu@intel.com>"

# default shell type
SHELL ["/bin/bash", "-c"]

# ignore the warning
ARG DEBIAN_FRONTEND=noninteractive
ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN apt-get update && apt-get install --assume-yes apt-utils

#update cmake
WORKDIR /tmp
RUN apt-get install -y wget && wget https://github.com/Kitware/CMake/releases/download/v3.14.4/cmake-3.14.4-Linux-x86_64.sh \
&& chmod +x cmake-3.14.4-Linux-x86_64.sh \
&& ./cmake-3.14.4-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir --skip-license \
&& rm ./cmake-3.14.4-Linux-x86_64.sh

# install openvino 2022.1
# https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_apt.html
RUN apt update && apt install --assume-yes curl gnupg2 lsb-release
RUN curl -s https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB |apt-key add -
RUN echo "deb https://apt.repos.intel.com/openvino/2022 $(lsb_release -cs) main" |tee /etc/apt/sources.list.d/intel-openvino-2022.list
RUN apt update && apt-cache search openvino-2022.1.0
RUN apt install --assume-yes openvino-2022.1.0
RUN ls -lh /opt/intel/openvino_2022
RUN source /opt/intel/openvino_2022/setupvars.sh 

# install librealsense2
RUN apt-get install -y --no-install-recommends \
software-properties-common 
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# Make sure you set http-proxy in below commands if your environment needs.
# RUN apt-key adv --keyserver-options http-proxy=your_proxy --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=your_proxy --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
RUN apt-key adv --keyserver-options http-proxy=${HTTP_PROXY} --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver-options http-proxy=${HTTP_PROXY} --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \ 
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
RUN pip install --upgrade pip
RUN pip install openvino-dev[tensorflow2]==2022.1
RUN apt-get install -y libcanberra-gtk* git

# build ros openvino toolkit
# set env before build ros openvino tolkkit
WORKDIR /root
RUN mkdir -p catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git init && git clone -b ros https://github.com/intel/ros_openvino_toolkit \
&& git clone https://github.com/intel/object_msgs.git \
&& git clone -b ${VERSION} https://github.com/ros-perception/vision_opencv.git \
&& git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
RUN apt-get install ros-${VERSION}-ddynamic-reconfigure
WORKDIR /root/catkin_ws/
RUN source /opt/ros/${VERSION}/setup.bash && source /opt/intel/openvino_2022/setupvars.sh && catkin_make_isolated
