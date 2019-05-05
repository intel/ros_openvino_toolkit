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
**Note**:You can choose to build the environment using *./environment_setup_binary.sh* script in the script subfolder. The *modules.conf* file in the same directory as the .sh file is the configuration file that controls the installation process.You can modify the *modules.conf* to customize your installation process.
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
git clone https://github.com/intel-ros/realsense
cd realsense
git checkout 2.1.3
```

- Build package
```
# Ubuntu 16.04
source /opt/ros/kinetic/setup.bash
# Ubuntu 18.04
source /opt/ros/melodic/setup.bash

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
		#object segmentation model
		 cd /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/install_prerequisites
		 sudo ./install_prerequisites.sh
		 mkdir -p ~/Downloads/models
		 cd ~/Downloads/models
		 wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
		 tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
		 cd mask_rcnn_inception_v2_coco_2018_01_28
		 python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir ./output/
		 sudo mkdir -p /opt/models
		 sudo ln -sf ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28 /opt/models/
		 #object detection model
		 cd /opt/intel/computer_vision_sdk/deployment_tools/model_downloader
		 sudo python3 ./downloader.py --name mobilenet-ssd
		 #FP32 precision model
		 sudo python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
		 #FP16 precision model
		 sudo python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
      ```
	* copy label files (excute _once_)<br>
		```bash
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/emotions-recognition-retail-0003/FP16
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP32
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/computer_vision_sdk/deployment_tools/intel_models/face-detection-adas-0001/FP16
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28/output
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16
		```
	* set ENV LD_LIBRARY_PATH and environment
		```bash
		source /opt/intel/computer_vision_sdk/bin/setupvars.sh
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/build/intel64/Release/lib
		```
* run face detection sample code input from StandardCamera.(connect Intel® Neural Compute Stick 2)
	```bash
	roslaunch vino_launch pipeline_people_myriad.launch
	```
* run face detection sample code input from Image.
	```bash
	roslaunch vino_launch pipeline_image.launch
	```
* run object detection sample code input from RealSenseCamera.
	```bash
	roslaunch vino_launch pipeline_object.launch
	```
* run object detection sample code input from RealSenseCameraTopic.(connect Intel® Neural Compute Stick 2)
	```bash
	roslaunch vino_launch pipeline_object_topic.launch
	```
* run object segmentation sample code input from RealSenseCameraTopic.
	```bash
	roslaunch vino_launch pipeline_segmentation.launch
	```
* run object segmentation sample code input from Video.
	```bash
	roslaunch vino_launch pipeline_video.launch
	```
* run person reidentification sample code input from StandardCamera.
	```bash
	roslaunch vino_launch pipeline_reidentification.launch
	```
* run object detection service sample code input from Image  
  Run image processing service:
	```bash
	roslaunch vino_launch image_object_server.launch
	```
  Run example application with an absolute path of an image on another console:
	```bash
	rosrun dynamic_vino_sample image_object_client ~/catkin_ws/src/ros_openvino_toolkit/data/images/car.png
	```
* run people detection service sample code input from Image  
  Run image processing service:
	```bash
	roslaunch vino_launch image_people_server.launch
	```
  Run example application with an absolute path of an image on another console:
	```bash
	rosrun dynamic_vino_sample image_people_client ~/catkin_ws/src/ros_openvino_toolkit/data/images/team.jpg
	```
## 6. Known Issues
* Possible problems
	* When running sample with Intel® Neural Compute Stick 2 occurred error:
		```bash
		E: [ncAPI] [         0] ncDeviceOpen:668	failed to find device
		# or
		E: [ncAPI] [         0] ncDeviceCreate:324      global mutex initialization failed
		```
	> solution - Please refer to the [guide](https://software.intel.com/en-us/neural-compute-stick/get-started) to set up the environment.
	* Segmentation fault occurs occasionally, which is caused by librealsense library. See [Issue #2645](https://github.com/IntelRealSense/librealsense/issues/2645) for more details.

###### *Any security issue should be reported using process at https://01.org/security*

