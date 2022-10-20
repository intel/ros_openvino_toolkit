# ROS_MELODIC_OpenVINO_Toolkit

**NOTE:** 
Below steps have been tested on **Ubuntu 18.04**.

## 1. Environment Setup
* Install ROS Melodic ([guide](http://wiki.ros.org/melodic/Installation/Ubuntu)).

* Install Intel® OpenVINO™ Toolkit Version: 2022.1 ([guide](https://docs.openvino.ai/2022.1/openvino_docs_install_guides_installing_openvino_linux.html)). 
  * Install from an achive file ([guide](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html)).
Tips: Both runtime and development tool are needed, `pip` is recommended for installing the development tool ([guide](https://docs.openvino.ai/latest/openvino_docs_install_guides_install_dev_tools.html)).
  * Install from source code ([guide](https://github.com/openvinotoolkit/openvino/wiki/BuildingForLinux)).

* Install Intel®  RealSense ™ SDK ([guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)).

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
git clone https://github.com/intel/ros_openvino_toolkit -b melodic
git clone https://github.com/intel/object_msgs
git clone https://github.com/ros-perception/vision_opencv.git -b melodic
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
```
* Install dependencies
```
sudo apt-get install ros-melodic-ddynamic-reconfigure
```
* Build package
```
source /opt/ros/melodic/setup.bash
source <OpenVINO_INSTALL_DIR>/setupvars.sh
cd ~/catkin_ws
catkin_make_isolated
source ./devel_isolated/setup.bash
```

## 3. Running the Demo
### Install OpenVINO 2022.1 by source code
* See all available models
```
cd <OpenVINO_SOURCECODE_DIR>/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once):
```
cd <OpenVINO_SOURCECODE_DIR>/openvino/thirdparty/open_model_zoo/tools/model_tools
sudo python3 downloader.py --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
sudo python3 downloader.py --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
sudo python3 downloader.py --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
sudo python3 downloader.py --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
sudo python3 downloader.py --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
sudo python3 downloader.py --name person-reidentification-retail-0277 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
sudo python3 downloader.py --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
sudo python3 downloader.py --name face-reidentification-retail-0095 --output_dir /opt/openvino_toolkit/models/face-reidentification-retail/output
sudo python3 downloader.py --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recognition/output
sudo python3 downloader.py --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
```

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi)need to be converted to intermediate representation (For example the model for object detection)
  * mobilenet-ssd (replaced ssd_mobilenet_v2_coco to mobilenet-ssd for object detection in ov2022)
  ```
  cd <OpenVINO_SOURCECODE_DIR>/openvino/thirdparty/open_model_zoo/tools/model_tools
  sudo python3 downloader.py --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output
  sudo python3 converter.py --name mobilenet-ssd -d /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/ -o /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/convert
  ```
  * vehicle-license-plate-detection-barrier-0123
  ```
  cd <OpenVINO_SOURCECODE_DIR>/openvino/thirdparty/open_model_zoo/tools/model_tools
  sudo python3 downloader.py --name vehicle-license-plate-detection-barrier-0123 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output
  sudo python3 converter.py --name vehicle-license-plate-detection-barrier-0123 -d /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output -o /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output/convert
  ```
  * deeplabv3
  ```
  cd <OpenVINO_SOURCECODE_DIR>/openvino/thirdparty/open_model_zoo/tools/model_tools
  sudo python3 downloader.py --name deeplabv3 --output_dir /opt/openvino_toolkit/models/deeplabv3/output
  sudo python3 converter.py --name deeplabv3 -d /opt/openvino_toolkit/models/deeplabv3/output -o /opt/openvino_toolkit/models/deeplabv3/output/convert
  ```

* Copy label files (execute once)
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
### Install OpenVINO 2022.1 by PIP
* OMZ tools are provided for downloading and converting OMZ models in ov2022 ([guide](https://pypi.org/project/openvino-dev/)).

* See all available models
```
omz_downloader --print_all
```

* Download the optimized Intermediate Representation (IR) of model (execute once), for example:
```
omz_downloader --name face-detection-adas-0001 --output_dir /opt/openvino_toolkit/models/face_detection/output
omz_downloader --name age-gender-recognition-retail-0013 --output_dir /opt/openvino_toolkit/models/age-gender-recognition/output
omz_downloader --name emotions-recognition-retail-0003 --output_dir /opt/openvino_toolkit/models/emotions-recognition/output
omz_downloader --name head-pose-estimation-adas-0001 --output_dir /opt/openvino_toolkit/models/head-pose-estimation/output
omz_downloader --name person-detection-retail-0013 --output_dir /opt/openvino_toolkit/models/person-detection/output
omz_downloader --name person-reidentification-retail-0277 --output_dir /opt/openvino_toolkit/models/person-reidentification/output
omz_downloader --name landmarks-regression-retail-0009 --output_dir /opt/openvino_toolkit/models/landmarks-regression/output
omz_downloader --name semantic-segmentation-adas-0001 --output_dir /opt/openvino_toolkit/models/semantic-segmentation/output
omz_downloader --name vehicle-attributes-recognition-barrier-0039 --output_dir /opt/openvino_toolkit/models/vehicle-attributes-recognition/output
omz_downloader --name license-plate-recognition-barrier-0001 --output_dir /opt/openvino_toolkit/models/license-plate-recognition/output
```
* Copy label files (execute once)
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

* If the model (tensorflow, caffe, MXNet, ONNX, Kaldi) need to be converted to intermediate representation (such as the model for object detection):
  * mobilenet-ssd (replaced ssd_mobilenet_v2_coco to mobilenet-ssd for object detection in ov2022)
    ```
    omz_downloader --name mobilenet-ssd --output_dir /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output
    omz_converter --name mobilenet-ssd -d /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output -o /opt/openvino_toolkit/models/object_detection/mobilenet_ssd/output/convert
    ```
  * vehicle-license-plate-detection-barrier-0123
    ```
    omz_downloader --name vehicle-license-plate-detection-barrier-0123 --output_dir /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output
    omz_converter --name vehicle-license-plate-detection-barrier-0123 -d /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output -o /opt/openvino_toolkit/models/vehicle-license-plate-detection-barrier/output/convert
    ```

  * deeplabv3
    ```
    omz_downloader --name deeplabv3 --output_dir /opt/openvino_toolkit/models/deeplabv3/output
    omz_converter --name deeplabv3 -d /opt/openvino_toolkit/models/deeplabv3/output -o /opt/openvino_toolkit/models/deeplabv3/output/convert
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



