# Introduction

The OpenVINO™ (Open visual inference and neural network optimization) toolkit provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference. By leveraging Intel® OpenVINO™ toolkit and corresponding libraries, this runtime framework extends  workloads across Intel® hardware (including accelerators) and maximizes performance.
* Enables CNN-based deep learning inference at the edge
* Supports heterogeneous execution across computer vision accelerators—CPU, GPU, Intel® Movidius™ Neural Compute Stick, and FPGA—using a common API
* Speeds up time to market via a library of functions and preoptimized kernels
* Includes optimized calls for OpenCV and OpenVX*

## Design Architecture
From the view of hirarchical architecture design, the package is divided into different functional components, as shown in below picture. 

![OpenVINO_Architecture](https://github.com/intel/ros_openvino_toolkit/blob/devel/data/images/design_arch.PNG "OpenVINO RunTime Architecture")

- **Intel® OpenVINO™ toolkit** is leveraged to provide deep learning basic implementation for data inference. is free software that helps developers and data scientists speed up computer vision workloads, streamline deep learning inference and deployments,
and enable easy, heterogeneous execution across Intel® platforms from edge to cloud. It helps to:
   - Increase deep learning workload performance up to 19x1 with computer vision accelerators from Intel.
   - Unleash convolutional neural network (CNN)-based deep learning inference using a common API.
   - Speed development using optimized OpenCV* and OpenVX* functions.
- **ros OpenVINO Runtime Framework** is the main body of this repo. it provides key logic implementation for pipeline lifecycle management, resource management and ROS system adapter, which extends Intel OpenVINO toolkit and libraries. Furthermore, this runtime framework provides ways to ease launching, configuration and data analytics and re-use.
- **Diversal Input resources** are the data resources to be infered and analyzed with the OpenVINO framework.
- **ROS interfaces and outputs** currently include _Topic_ and _service_. Natively, RViz output and CV image window output are also supported by refactoring topic message and inferrence results.
- **Optimized Models** provides by Model Optimizer component of Intel® OpenVINO™ toolkit. Imports trained models from various frameworks (Caffe*, Tensorflow*, MxNet*, ONNX*, Kaldi*) and converts them to a unified intermediate representation file. It also optimizes topologies through node merging, horizontal fusion, eliminating batch normalization, and quantization.It also supports graph freeze and graph summarize along with dynamic input freezing.

## Logic Flow
From the view of logic implementation, the package introduces the definitions of parameter manager, pipeline and pipeline manager. The below picture depicts how these entities co-work together when the corresponding program is launched.

![Logic_Flow](https://github.com/intel/ros_openvino_toolkit/blob/devel/data/images/impletation_logic.PNG "OpenVINO RunTime Logic Flow")

Once a corresponding program is launched with a specified .yaml config file passed in the .launch file or via commandline, _**parameter manager**_ analyzes the configurations about pipeline and the whole framework, then shares the parsed configuration information with pipeline procedure. A _**pipeline instance**_ is created by following the configuration info and is added into _**pipeline manager**_ for lifecycle control and inference action triggering.

The contents in **.yaml config file** should be well structured and follow the supported rules and entity names. Please see [the configuration guidance](https://github.com/intel/ros_openvino_toolkit/blob/devel/doc/YAML_CONFIGURATION_GUIDE.md) for how to create or edit the config files.

**Pipeline** fulfills the whole data handling process: initiliazing Input Component for image data gathering and formating; building up the structured inference network and passing the formatted data through the inference network; transfering the inference results and handling output, etc.

**Pipeline manager** manages all the created pipelines according to the inference requests or external demands (say, system exception, resource limitation, or end user's operation). Because of co-working with resource management and being aware of the whole framework, it covers the ability of performance optimization by sharing system resource between pipelines and reducing the burden of data copy.

# Supported Features
## Diversal Input Components
Currently, the package support several kinds of input resources of gaining image data:

|Input Resource|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|Any RGB camera with USB port supporting. Currently only the first USB camera if many are connected.|
|RealSenseCamera| Intel RealSense RGB-D Camera, directly calling RealSense Camera via librealsense plugin of openCV.|
|Image Topic| any ROS topic which is structured in image message.|
|Image File| Any image file which can be parsed by openCV, such as .png, .jpeg.|
|Video File| Any video file which can be parsed by openCV.|

## Inference Implementations
Currently, the inference feature list is supported:

|Inference|Description|
|-----------------------|------------------------------------------------------------------|
|Face Detection|Object Detection task applied to face recognition using a sequence of neural networks.|
|Emotion Recognition| Emotion recognition based on detected face image.|
|Age & Gender Recognition| Age and gener recognition based on detected face image.|
|Head Pose Estimation| Head pose estimation based on detected face image.|
|Object Detection| object detection based on SSD-based trained models.|
|Vehicle Detection| Vehicle and passenger detection based on Intel models.|
|Object Segmentation| object detection and segmentation.|
|Person Reidentification| Person Reidentification based on object detection.|

## ROS interfaces and outputs
### Topic
#### Subscribed Topic
- Image topic:
```/camera/color/image_raw```([sensor_msgs::Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
#### Published Topic
- Face Detection:
```/ros_openvino_toolkit/face_detection```([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Emotion Recognition:
```/ros_openvino_toolkit/emotions_recognition```([people_msgs::EmotionsStamped](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/msg/EmotionsStamped.msg))
- Age and Gender Recognition:
```/ros_openvino_toolkit/age_genders_Recognition```([people_msgs::AgeGenderStamped](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/msg/AgeGenderStamped.msg))
- Head Pose Estimation:
```/ros_openvino_toolkit/headposes_estimation```([people_msgs::HeadPoseStamped](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/msg/HeadPoseStamped.msg))
- Object Detection:
```/ros_openvino_toolkit/detected_objects```([object_msgs::ObjectsInBoxes](https://github.com/intel/object_msgs/blob/master/msg/ObjectsInBoxes.msg))
- Object Segmentation:
```/ros_openvino_toolkit/segmented_obejcts```([people_msgs::ObjectsInMasks](https://github.com/intel/ros_openvino_toolkit/blob/devel/people_msgs/msg/ObjectsInMasks.msg))
- Person Reidentification:
```/ros_openvino_toolkit/reidentified_persons```([people_msgs::ReidentificationStamped](https://github.com/intel/ros_openvino_toolkit/blob/devel/people_msgs/msg/ReidentificationStamped.msg))
- Vehicle Detection:
```/ros_openvino_toolkit/detected_license_plates```([people_msgs::VehicleAttribsStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/VehicleAttribsStamped.msg)
- Vehicle License Detection:
```/ros_openvino_toolkit/detected_license_plates```([people_msgs::LicensePlateStamped](https://github.com/intel/ros2_openvino_toolkit/blob/devel/people_msgs/msg/LicensePlateStamped.msg)
- Rviz Output:
```/ros_openvino_toolkit/image_rviz```([sensor_msgs::Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))

### Service
- Object Detection Service:
```/detect_object``` ([object_msgs::DetectObject](https://github.com/intel/object_msgs/blob/master/srv/DetectObject.srv))
- Face Detection Service:
```/detect_face``` ([object_msgs::DetectObject](https://github.com/intel/object_msgs/blob/master/srv/DetectObject.srv))
- Age & Gender Detection Service:
```/detect_age_gender``` ([people_msgs::AgeGender](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/srv/AgeGenderSrv.srv))
- Headpose Detection Service:
```/detect_head_pose``` ([people_msgs::HeadPose](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/srv/HeadPoseSrv.srv))
- Emotion Detection Service:
```/detect_emotion``` ([people_msgs::Emotion](https://github.com/intel/ros_openvino_toolkit/blob/master/people_msgs/srv/EmotionSrv.srv))


### RViz
RViz dispaly is also supported by the composited topic of original image frame with inference result.
To show in RViz tool, add an image marker with the composited topic:
```/ros_openvino_toolkit/image_rviz```([sensor_msgs::Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))

### Image Window
OpenCV based image window is natively supported by the package.
To enable window, Image Window output should be added into the output choices in .yaml config file. see [the config file guidance](https://github.com/intel/ros_openvino_toolkit/blob/devel/doc/YAML_CONFIGURATION_GUIDE.md) for checking/adding this feature in your launching.

## Demo Result Snapshots
See below pictures for the demo result snapshots.
* face detection input from standard camera
![face_detection_demo_image](https://github.com/intel/ros_openvino_toolkit/blob/devel/data/images/face_detection.png "face detection demo image")

* object detection input from realsense camera
![object_detection_demo_realsense](https://github.com/intel/ros_openvino_toolkit/blob/devel/data/images/object_detection.gif "object detection demo realsense")

* object segmentation input from video
![object_segmentation_demo_video](https://github.com/intel/ros_openvino_toolkit/blob/devel/data/images/object_segmentation.gif "object segmentation demo video")

* Person Reidentification input from standard camera
![person_reidentification_demo_video](https://github.com/intel/ros2_openvino_toolkit/blob/devel/data/images/person-reidentification.gif "person reidentification demo video")

# Installation & Launching
**NOTE:** Intel releases 2 different series of OpenVINO Toolkit, we call them as [OpenSource Version](https://github.com/opencv/dldt/) and [Tarball Version](https://software.intel.com/en-us/openvino-toolkit). This guidelie uses OpenSource Version as the installation and launching example. **If you want to use Tarball version, please follow [the guide for Tarball Version](https://github.com/intel/ros_openvino_toolkit/blob/devel/doc/BINARY_VERSION_README.md).**

## Dependencies Installation
One-step installation scripts are provided for the dependencies' installation. Please see [the guide](https://github.com/intel/ros_openvino_toolkit/blob/devel/doc/OPEN_SOURCE_CODE_README.md) for details.

## Launching
* Preparation
	* Configure the Neural Compute Stick USB Driver
		```bash
		cd ~/Downloads
		cat <<EOF > 97-usbboot.rules
		SUBSYSTEM=="usb", ATTRS{idProduct}=="2150", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
		SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
		SUBSYSTEM=="usb", ATTRS{idProduct}=="f63b", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
		EOF
		sudo cp 97-usbboot.rules /etc/udev/rules.d/
		sudo udevadm control --reload-rules
		sudo udevadm trigger
		sudo ldconfig
		rm 97-usbboot.rules
		```
	* download [Object Detection model](https://github.com/intel/ros_openvino_toolkit/tree/devel/doc/OBJECT_DETECTION.md)
	* download and convert a trained model to produce an optimized Intermediate Representation (IR) of the model 
		```bash
		#object segmentation model
		cd /opt/openvino_toolkit/dldt/model-optimizer/install_prerequisites
		sudo ./install_prerequisites.sh
		mkdir -p ~/Downloads/models
		cd ~/Downloads/models
		wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
		tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
		cd mask_rcnn_inception_v2_coco_2018_01_28
		#FP32
		sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir /opt/openvino_toolkit/models/segmentation/output/FP32
		#FP16
		sudo python3 /opt/openvino_toolkit/dldt/model-optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/openvino_toolkit/dldt/model-optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --data_type=FP16 --output_dir /opt/openvino_toolkit/models/segmentation/output/FP16
		```
	* download the optimized Intermediate Representation (IR) of model (excute _once_)<br>
		```bash
		cd /opt/openvino_toolkit/open_model_zoo/tools/downloader
		sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
		sudo python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
		sudo python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
		sudo python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
		sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
		sudo python3 downloader.py --name person-reidentification-retail-0076 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
		sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0106 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection/output
		sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recongnition/output
		sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
		sudo python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
		sudo python3 downloader.py --name face-reidentification-retail-0095 --output_dir /opt/openvino_toolkit/models/face-reidentification/output
		```
	* copy label files (excute _once_)<br>
		```bash
		sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP32/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP16/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/intel/vehicle-license-plate-detection-barrier-0106/FP32
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
	  sudo cp /opt/openvino_toolkit/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/

		```
	* set ENV LD_LIBRARY_PATH<br>
		```bash
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openvino_toolkit/dldt/inference-engine/bin/intel64/Release/lib
		```
* run face detection sample code input from StandardCamera.
	```bash
	roslaunch vino_launch pipeline_people.launch
	```
* run face detection sample code input from Image.
	```bash
	roslaunch vino_launch pipeline_image.launch
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
* run face re-identification with facial landmarks from realsense camera
	```bash
	roslaunch vino_launch pipeline_face_reidentification.launch
	```
* run vehicle detection sample code input from StandardCamera.
	```bash
	roslaunch vino_launch pipeline_vehicle_detection.launch  
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
* run face detection service sample code input from Image  
  Run image processing service:
	```bash
	roslaunch vino_launch image_people_server.launch
	```
  Run example application with an absolute path of an image on another console:
	```bash
	rosrun dynamic_vino_sample image_people_client ~/catkin_ws/src/ros_openvino_toolkit/data/images/team.jpg
	```
# TODO Features
* Support **result filtering** for inference process, so that the inference results can be filtered to different subsidiary inference. For example, given an image, firstly we do Object Detection on it, secondly we pass cars to vehicle brand recognition and pass license plate to license number recognition.
* Design **resource manager** to better use such resources as models, engines, and other external plugins.
* Develop GUI based **configuration and management tools** (and monitoring and diagnose tools), in order to provide easy entry for end users to simplify their operation. 

# More Information
* ros OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw 


