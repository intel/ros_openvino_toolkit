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
./environment_setup_binary.sh username password
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

- Install Intel® RealSense™ SDK 2.0 [(tag v2.17.1)](https://github.com/IntelRealSense/librealsense/tree/v2.17.1)
	* [Install from source code](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)(Recommended)
	* [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)

- Other Dependencies
```bash
# numpy
pip3 install numpy
# libboost
sudo apt-get install -y --no-install-recommends libboost-all-dev
cd /usr/lib/x86_64-linux-gnu
sudo ln -s libboost_python-py35.so libboost_python3.so
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
cd ~/catkin_ws
catkin_make
```

## 5. Running the Demo
* Please go to [README](https://github.com/intel/ros_openvino_toolkit/blob/devel/doc/OPEN_SOURCE_CODE_README.md)

## 6. Known Issues
- Video and image detection support is going to be verified.
- Segmentation fault occurs occasionally, which is caused by librealsense library. See [Issue #2645](https://github.com/IntelRealSense/librealsense/issues/2645) for more details.

###### *Any security issue should be reported using process at https://01.org/security*
