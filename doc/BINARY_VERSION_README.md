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
- ROS Kinetic

- [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit)
- RGB Camera, e.g. RealSense D400 Series or standard USB camera or Video/Image File
- Graphics are required only if you use a GPU. The official system requirements for GPU are:
  * 6th to 8th generation Intel® Core™ processors with Iris® Pro graphics and Intel® HD Graphics
  * 6th to 8th generation Intel® Xeon® processors with Iris Pro graphics and Intel HD Graphics (excluding the e5 product family, which does not have graphics)
  * Intel® Pentium® processors N4200/5, N3350/5, N3450/5 with Intel HD Graphics

  Use one of the following methods to determine the GPU on your hardware:
  1. [lspci] command: GPU info may lie in the [VGA compatible controller] line.
  2. Ubuntu system: Menu [System Settings] --> [Details] may help you find the graphics information.
  3. Openvino: Download the install package, install_GUI.sh inside will check the GPU information before installation.

## 3. Environment Setup
**Note**:You can choose to build the environment using *./environment_setup_binary.sh* script in the script subfolder.
```bash
./environment_setup_binary.sh
```
**Note**:You can also choose to follow the steps below to build the environment step by step.

- Install ROS Kinetic Desktop-Full ([guide](http://wiki.ros.org/kinetic/Installation/Ubuntu))

- Install [OpenVINO™ Toolkit](https://software.intel.com/en-us/openvino-toolkit) ([guide](https://software.intel.com/en-us/articles/OpenVINO-Install-Linux)). Choose "2018 R4" when download tarball.

	**Note**: Please use  *root privileges* to run the installer when installing the core components.
- Install OpenCL Driver for GPU
```bash
cd /opt/intel/computer_vision_sdk/install_dependencies
sudo ./install_NEO_OCL_driver.sh
```
* Install [OpenCV 3.4 or later](https://docs.opencv.org/master/d9/df8/tutorial_root.html)([guide](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html))
	```bash
	[compiler] sudo apt-get install build-essential
	[required] sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
	[optional] sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev 
	mkdir -p ~/code && cd ~/code
	git clone https://github.com/opencv/opencv.git
	git clone https://github.com/opencv/opencv_contrib.git
	cd opencv && git checkout 3.4.2 && cd ..
	cd opencv_contrib && git checkout 3.4.2 && cd ..
	cd opencv
	mkdir build && cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/home/<hostname>/code/opencv_contrib/modules/ ..
	make -j8
	sudo make install
	```
	* Additional steps are required on ubuntu 18.04
		```bash
		sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
		sudo apt update
		sudo apt install libjasper1 libjasper-dev
		```
- Install Intel® RealSense™ SDK 2.0 [(tag v2.17.1)](https://github.com/IntelRealSense/librealsense/tree/v2.17.1)
	* [Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)(Recommended)
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)

- Other Dependencies
```bash
# numpy and networkx
pip3 install numpy
pip3 install networkx
# libboost
sudo apt-get install -y --no-install-recommends libboost-all-dev
cd /usr/lib/x86_64-linux-gnu
sudo ln -sf libboost_python-py35.so libboost_python3.so
```

## 4. Building and Installation
- Build sample code under openvino toolkit
```bash
# root is required instead of sudo
source /opt/intel/computer_vision_sdk/bin/setupvars.sh
cd /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/
mkdir build
cd build
cmake ..
make
```

- Set Environment CPU_EXTENSION_LIB and GFLAGS_LIB
```bash
export CPU_EXTENSION_LIB=/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a
```

- Install ROS_OpenVINO packages
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_openvino_toolkit
git clone https://github.com/intel/object_msgs
git clone https://github.com/ros-perception/vision_opencv
```

- Build package
```
source /opt/ros/kinetic/setup.bash
source /opt/intel/computer_vision_sdk/bin/setupvars.sh
export OpenCV_DIR=$HOME/code/opencv/build
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
sudo mkdir -p /opt/openvino_toolkit
sudo ln -s ~/catkin_ws/src/ros_openvino_toolkit /opt/openvino_toolkit/ros_openvino_toolkit
```

## 5. Running the Demo
* Preparation
	* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
		```bash
		cd /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/install_prerequisites
		sudo ./install_prerequisites.sh
		#object detection model
		cd /opt/intel/computer_vision_sdk/deployment_tools/model_downloader
		sudo python3 downloader.py --name ssd300
		sudo python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/ssd/300/caffe/ssd300.caffemodel --output_dir /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/ssd/300/caffe/output/
		```
	* copy label files (excute _once_)<br>
		```bash
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/ssd300.labels /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/ssd/300/caffe/output
		```
	* set ENV LD_LIBRARY_PATH
		```bash
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
		```
* run face detection sample code input from StandardCamera.
	```bash
	roslaunch vino_launch pipeline_people.launch
	```
* run object detection sample code input from RealsensCamera.
	```bash
	roslaunch vino_launch pipeline_object.launch
	```
## 6. Known Issues
- Video and image detection support is going to be verified.
- Segmentation fault occurs occasionally, which is caused by librealsense library. See [Issue #2645](https://github.com/IntelRealSense/librealsense/issues/2645) for more details.

###### *Any security issue should be reported using process at https://01.org/security*
