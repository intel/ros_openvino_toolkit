# ROS_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 18.04** and **Ubuntu 20.04**.
Supported ROS versions include noetic and melodic.

## 1. Environment Setup
* For ROS noetic on ubuntu 20.04:
  * Install ROS. ([noetic_guide](http://wiki.ros.org/noetic/Installation/Ubuntu))

  * Install Intel® OpenVINO™ Toolkit Version: 2022.1. ([guide](https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_linux.html)) 
    * Install from an achive file. Both runtime and development tool are needed, `pip` is recommended for installing the development tool. ([guide](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html)) 

  * Install Intel® RealSense™ SDK. ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))

* For ROS melodic on ubuntu 18.04:
  * Install ROS. ([melodic_guide](http://wiki.ros.org/melodic/Installation/Ubuntu))

  * Install Intel® OpenVINO™ Toolkit Version: 2022.1. ([guide](https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_linux.html)) 
    * Install from an achive file. Both runtime and development tool are needed, `pip` is recommended for installing the development tool. ([guide](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html)) 

  * Install Intel® RealSense™ SDK. ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))

* Install Dependency
  * Install gflags 
  ```
    sudo apt-get install -y libgflags-dev
  ```

## 2. Build and Installation
* Install ROS_OpenVINO_Toolkit packages
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/intel/ros_openvino_toolkit -b ros
git clone https://github.com/intel/object_msgs
git clone https://github.com/ros-perception/vision_opencv.git -b <ROS_VERSION>
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
```
* Install dependencies
```
sudo apt-get install ros-<ROS_VERSION>-dynamic-reconfigure
sudo apt install python3-colcon-common-extensions
```
* Build package
```
source /opt/ros/<ROS_VERSION>/setup.bash
source <OpenVINO_INSTALL_DIR>/setupvars.sh
cd ~/catkin_ws
catkin_make_isolated
source ./devel_isolated/setup.bash
```

## 3. Running the Demo
### Install OpenVINO 2022.1 by PIP
* OMZ tools are provided for downloading and converting models of open_model_zoo in ov2022.([guide](https://pypi.org/project/openvino-dev/))

* See all available models
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
omz_converter --list convert_model.lst -o /opt/openvino_toolkit/models/convert
```
### Install OpenVINO 2022.1 by source code
* See all available models
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of models (execute once), for example:
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --list ~/catkin_ws/src/ros_openvino_toolkit/data/model_list/download_model.lst -o /opt/openvino_toolkit/models/
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to Intermediate Representation (such as the model for object detection):
```
cd ~/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 converter.py --list ~/catkin_ws/src/ros_openvino_toolkit/data/model_list/convert_model.lst -o /opt/openvino_toolkit/models/convert
```

* Copy label files (execute once)
**Note**:Need to make label_dirs if skip steps for set output_dirs above.
* Before launch, copy label files to the same model path, make sure the model path and label path match the ros_openvino_toolkit/vino_launch/param/xxxx.yaml.
```
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP32/
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/intel/emotions-recognition-retail-0003/FP16/
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP32/
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/intel/face-detection-adas-0001/FP16/
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/convert/intel/vehicle-license-plate-detection-barrier-0106/FP32
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/convert/public/mobilenet-ssd/FP16
 sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/deeplabv3/output/convert/public/deeplabv3/FP16
 sudo mv /opt/openvino_toolkit/models/deeplabv3/output/convert/public/deeplabv3/FP16/frozen_inference_graph.labels  /opt/openvino_toolkit/models/deeplabv3/output/convert/public/deeplabv3/FP16/deeplabv3.labels
```

* Please check the parameter configuration in ros_openvino_toolkit/sample/param/xxxx.yaml before lauching, make sure parameters such as model_path, label_path and input_path are set correctly.
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

###### *Any security issue should be reported using process at https://01.org/security*



