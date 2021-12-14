# ROS_NOETIC_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 20.04**.

## 1. Environment Setup
* Install ROS Noetic ([guide](http://wiki.ros.org/noetic/Installation/Ubuntu))
* Install Intel® OpenVINO™ Toolkit ([guide](https://docs.openvino.ai/2021.4/openvino_docs_install_guides_installing_openvino_linux.html))
* Install Intel®  RealSense ™ SDK ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))
* Install Dependency
  * Install gflags 
  ```
    sudo apt-get install -y libgflags-dev
  ```

## 2. Building and Installation
* Install ROS_OpenVINO_Toolkit packages
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_openvino_toolkit -b dev-ov2021.4
git clone https://github.com/intel/object_msgs
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
```
* Install dependencies
```
sudo apt-get install ros-noetic-ddynamic-reconfigure
```
* Build package
```
source /opt/ros/noetic/setup.bash
source /opt/intel/openvino_2021/bin/setupvars.sh
cd ~/catkin_ws
catkin_make_isolated
source ./devel_isolated/setup.bash
```

## 3. Running the Demo
* Preparation
```
source /opt/intel/openvino_2021/bin/setupvars.sh
sudo mkdir -p /opt/openvino_toolkit
sudo ln -s /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader /opt/openvino_toolkit/models
sudo chmod 777 -R /opt/openvino_toolkit/models
```

* See all available models
```
cd /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once):
```
cd /opt/openvino_toolkit/models/
sudo python3 downloader/downloader.py --name face-detection-adas-0001
sudo python3 downloader/downloader.py --name age-gender-recognition-retail-0013
sudo python3 downloader/downloader.py --name emotions-recognition-retail-0003
sudo python3 downloader/downloader.py --name head-pose-estimation-adas-0001
sudo python3 downloader/downloader.py --name person-detection-retail-0013
sudo python3 downloader/downloader.py --name person-reidentification-retail-0277
sudo python3 downloader/downloader.py --name landmarks-regression-retail-0009
sudo python3 downloader/downloader.py --name face-reidentification-retail-0095
sudo python3 downloader/downloader.py --name vehicle-attributes-recognition-barrier-0039
sudo python3 downloader/downloader.py --name license-plate-recognition-barrier-0001
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi)need to be converted to intermediate representation (For example the model for object detection)
  * ssd_mobilenet_v2_coco
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name ssd_mobilenet_v2_coco
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=ssd_mobilenet_v2_coco --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py
  ```
  * vehicle-license-plate-detection-barrier-0123
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name vehicle-license-plate-detection-barrier-0123
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=vehicle-license-plate-detection-barrier-0123 --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py 
  ```
  * deeplabv3
  ```
  cd /opt/openvino_toolkit/models/
  sudo python3 downloader/downloader.py --name deeplabv3
  sudo python3 /opt/intel/openvino_2021/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=deeplabv3 --mo /opt/intel/openvino_2021/deployment_tools/model_optimizer/mo.py 
  ```

* copy label files (execute once)
```
 cd /opt/openvino_toolkit/models
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels intel/emotions-recognition-retail-0003/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels public/vehicle-license-plate-detection-barrier-0123/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels public/mobilenet-ssd/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels public/deeplabv3/FP16/
mv public/deeplabv3/FP16/frozen_inference_graph.labels  public/deeplabv3/FP16/deeplabv3.labels
```

* Before launch, check the parameter configuration in ros_openvino_toolkit/vino_launch/param/xxxx.yaml, make sure the paramter like model path, label path, inputs are right.
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
  * run object segmentation sample code input from RealSenseCameraTopic.
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

###### *Any security issue should be reported using process at https://01.org/security*


