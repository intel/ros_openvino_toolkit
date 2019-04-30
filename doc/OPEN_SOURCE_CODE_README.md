# ros_openvino_toolkit

## 1. Introduction
The [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) toolkit quickly deploys applications and solutions that emulate human vision. Based on Convolutional Neural Networks (CNN), the Toolkit extends computer vision (CV) workloads across Intel® hardware, maximizing performance.

This project is a ROS wrapper for CV API of [OpenVINO™](https://software.intel.com/en-us/openvino-toolkit), providing the following features:
* Support CPU and GPU platforms
* Support standard USB camera and Intel® RealSense™ camera
* Face detection
* Emotion recognition
* Age and gender recognition
* Head pose recognition
* Demo application to show above detection and recognitions

## 2. Prerequisite
- An x86_64 computer running Ubuntu 16.04. Below processors are supported:
	* 6th-8th Generation Intel® Core™
	* Intel® Xeon® v5 family
	* Intel®  Xeon® v6 family
- ROS Kinetic(For Ubuntu16.04) or ROS Melodic(For Ubuntu18.04)

- OpenVINO™ Toolkit Open Source
  	* The [Deep Learning Deployment Toolkit](https://github.com/opencv/dldt) that helps to enable fast, heterogeneous deep learning inferencing for Intel® processors (CPU and GPU/Intel® Processor Graphics), and supports more than 100 public and custom models.
	* [Open Model Zoo](https://github.com/opencv/open_model_zoo) includes 20+ pre-trained deep learning models to expedite development and improve deep learning inference on Intel® processors (CPU, Intel Processor Graphics, FPGA, VPU), along with many samples to easily get started.
- RGB Camera, e.g. RealSense D400 Series or standard USB camera or Video/Image File

- Graphics are required only if you use a GPU. The official system requirements for GPU are:
	* 6th to 8th generation Intel® Core™ processors with Iris® Pro graphics and Intel® HD Graphics
	* 6th to 8th generation Intel® Xeon® processors with Iris Pro graphics and Intel HD Graphics (excluding the e5 product family, which does not have graphics)
	* Intel® Pentium® processors N4200/5, N3350/5, N3450/5 with Intel HD Graphics

- Use one of the following methods to determine the GPU on your hardware:
	* [lspci] command: GPU info may lie in the [VGA compatible controller] line.
	* Ubuntu system: Menu [System Settings] --> [Details] may help you find the graphics information.
	* Openvino: Download the install package, install_GUI.sh inside will check the GPU information before installation.

## 3. Environment Setup
**Note**:You can choose to build the environment using *./environment_setup.sh* script in the script subfolder. The *modules.conf* file in the same directory as the .sh file is the configuration file that controls the installation process.You can modify the *modules.conf* to customize your installation process.
```bash
./environment_setup.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.
- For Ubuntu16.04, install ROS Kinetic Desktop-Full [(guide)](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- For Ubuntu18.04, install ROS Melodic Desktop-Full [(guide)](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Install OpenVINO™ Toolkit Open Source
	* Install [OpenCV 3.x: 3.4 or later](https://docs.opencv.org/master/d9/df8/tutorial_root.html)([guide](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html))
	```
	[compiler] sudo apt-get install build-essential
	[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
	[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
	cd ~/code
	git clone https://github.com/opencv/opencv.git
	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv && git checkout 3.4.2 && cd ..
	cd opencv_contrib && git checkout 3.4.2 && cd ..
	cd opencv
	mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=$HOME/code/opencv_contrib/modules/ ..
	make -j8
	sudo make install
	```

	* Install OpenCL Driver for GPU([guide](http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/intel-opencl-4.1-installation.pdf))<br>
	```bash
	cd ~/Downloads
	wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
	unzip SRB5.0_linux64.zip -d SRB5.0_linux64
	cd SRB5.0_linux64
	sudo apt-get install xz-utils
	mkdir intel-opencl
	tar -C intel-opencl -Jxf intel-opencl-r5.0-63503.x86_64.tar.xz
	tar -C intel-opencl -Jxf intel-opencl-devel-r5.0-63503.x86_64.tar.xz
	tar -C intel-opencl -Jxf intel-opencl-cpu-r5.0-63503.x86_64.tar.xz
	sudo cp -R intel-opencl/* /
	sudo ldconfig
	```
	* Install [Deep Learning Deployment Toolkit](https://github.com/opencv/dldt)([guide](https://github.com/opencv/dldt/tree/2018/inference-engine))<br>
	```bash
	mkdir ~/code && cd ~/code
	git clone https://github.com/opencv/dldt.git
	cd dldt/inference-engine/
	git checkout 2018_R4
	./install_dependencies.sh
	mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make -j8
	sudo mkdir -p /opt/openvino_toolkit
	sudo ln -s ~/code/dldt /opt/openvino_toolkit/dldt
	```
	* Install [Open Model Zoo](https://github.com/opencv/open_model_zoo)([guide](https://github.com/opencv/open_model_zoo/tree/2018/demos))<br>
	```bash
	cd ~/code
	git clone https://github.com/opencv/open_model_zoo.git
	cd open_model_zoo/demos/
	git checkout 2018_R4
	mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release /opt/openvino_toolkit/dldt/inference-engine
	make -j8
	sudo mkdir -p /opt/openvino_toolkit
	sudo ln -s ~/code/open_model_zoo /opt/openvino_toolkit/open_model_zoo
	```

- Install Intel® RealSense™ SDK 2.0 [(tag v2.17.1)](https://github.com/IntelRealSense/librealsense/tree/v2.17.1)<br>
	* [Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)(Recommended)<br>
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)<br>

- Other Dependencies
	```bash
	# numpy and networkx
	pip3 install numpy
	pip3 install networkx
	# libboost
	sudo apt-get install -y --no-install-recommends libboost-all-dev
	cd /usr/lib/x86_64-linux-gnu
	sudo ln -s libboost_python-py35.so libboost_python3.so
	```

## 4. Building and Installation

* set ENV InferenceEngine_DIR, CPU_EXTENSION_LIB and GFLAGS_LIB
	```bash
	export InferenceEngine_DIR=/opt/openvino_toolkit/dldt/inference-engine/build/
	export CPU_EXTENSION_LIB=/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libcpu_extension.so
	export GFLAGS_LIB=/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib/libgflags_nothreads.a
	```
* Install ROS_OpenVINO packages
	```bash
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	git clone https://github.com/intel/ros_openvino_toolkit
	git clone https://github.com/intel/object_msgs
	git clone https://github.com/ros-perception/vision_opencv
	git clone https://github.com/intel-ros/realsense
	cd realsense
	git checkout 2.1.3
	```

* Build package
	```
	# Ubuntu 16.04
	source /opt/ros/kinetic/setup.bash
	# Ubuntu 18.04
	source /opt/ros/melodic/setup.bash
	
	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash
	sudo mkdir -p /opt/openvino_toolkit
	sudo ln -s ~/catkin_ws/src/ros_openvino_toolkit /opt/openvino_toolkit/ros_openvino_toolkit
	```

###### *Any security issue should be reported using process at https://01.org/security*
