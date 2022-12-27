# Run Docker Images For ROS_OpenVINO_Toolkit

**NOTE:**
Below steps have been tested on **Ubuntu 20.04** and **Ubuntu 18.04** .
Supported ros versions include noetic and melodic.

## 1. Environment Setup
* Install docker ([guide](https://docs.docker.com/engine/install/ubuntu/))

## 2. Build docker image by dockerfile
```
cd ~/ros_openvino_toolkit/docker/Dockerfile
vi ~/ros_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_PRE_INSTALLED_PKG=<EXPECT_ROS_PRE_INSTALLED_PKG> --build-arg VERSION=<EXPECT_ROS_VERSION> --build-arg "HTTP_PROXY=set_your_proxy" -t ros_openvino_202201 .
```
For example:
* Build image for ros_noetic
```
cd ~/ros_openvino_toolkit/docker/Dockerfile
vi ~/ros_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_PRE_INSTALLED_PKG=noetic-desktop-full --build-arg VERSION=noetic --build-arg "HTTP_PROXY=set_your_proxy" -t ros_noetic_openvino_202201 .
```
* Build image for ros_melodic
```
cd ~/ros_openvino_toolkit/docker/Dockerfile
vi ~/ros_openvino_toolkit/docker/Dockerfile
docker build --build-arg ROS_PRE_INSTALLED_PKG=melodic-desktop-full --build-arg VERSION=melodic --build-arg "HTTP_PROXY=set_your_proxy" -t ros_melodic_openvino_202201 .
```

## 3. Download and load docker image
* Download docker image
```
 # ros_openvino_202201 for demo
 cd ~/Downloads/
 wget <DOCKER_IMAGE_PATH>
```
* Load docker image
```
cd ~/Downloads/
docker load -i <DOCKER_IMAGE>
docker images
// (show <DOCKER_IMAGE> in the list)
```

## 4. Running the Demos
* Install dependency
```
  sudo apt install x11-xserver-utils
  xhost +
```
* Run docker image
```
  docker images
  docker run -itd  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev  --privileged=true --name <your_image_name> <IMAGE_ID>
```
* In Docker Container

* Preparation
```
source /opt/intel/openvino_2022/setupvars.sh
source /opt/ros/<ROS_VERSION>/setup.bash
cd ~/catkin_ws
source ./devel_isolated/setup.bash
```

* See all available models
OMZ tools are provided for downloading and converting OMZ models in ov2022.([guide](https://pypi.org/project/openvino-dev/))

```
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
cd ~/catkin_ws/src/ros_openvino_toolkit/data/model_list
omz_downloader --list download_model.lst -o /opt/openvino_toolkit/models/
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
```
cd ~/catkin_ws/src/ros_openvino_toolkit/data/model_list
omz_converter --list convert_model.lst -d /opt/openvino_toolkit/models/ -o /opt/openvino_toolkit/models/convert
```
* Copy label files (execute once)
**Note**:Need to make label_dirs if skip steps for set output_dirs above.
```
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP32/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP32/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/intel/vehicle-license-plate-detection-barrier-0106/FP32
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/convert/public/mobilenet-ssd/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/convert/public/deeplabv3/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/intel/semantic-segmentation-adas-0001/FP16/
```

* Before launch, check the parameter configuration in ros_openvino_toolkit/sample/param/xxxx.yaml, make sure the paramter like model path, label path, inputs are right.
  * run face detection sample code input from StandardCamera.
  ```
  roslaunch vino_launch pipeline_people.launch
  ```
  * run person reidentification sample code input from StandardCamera.
  ```
  roslaunch vino_launch pipeline_face_reidentification.launch
  ```
  * run person reidentification sample code input from StandardCamera
  ```
  roslaunch vino_launch pipeline_reidentification.launch
  ```
  * run face detection sample code input from Image.
  ```
  roslaunch vino_launch pipeline_image.launch
  ```
  * run object sample
  ```
  roslaunch vino_launch pipeline_object.launch
  ```
  * run object topic sample
  ```
  roslaunch vino_launch pipeline_object_topic.launch
  ```
  * run object segmentation sample code input from RealSenseCamera.
  ```
  roslaunch vino_launch pipeline_segmentation.launch
  ```
  * run vehicle detection sample code input from StandardCamera.
  ```
  roslaunch vino_launch pipeline_vehicle_detection.launch
  ```
  * run video sample
  ```
  roslaunch vino_launch pipeline_video.launch
  ```

# More Information
* ros OpenVINO discription writen in Chinese: https://mp.weixin.qq.com/s/BgG3RGauv5pmHzV_hkVAdw

###### *Any security issue should be reported using process at https://01.org/security*

