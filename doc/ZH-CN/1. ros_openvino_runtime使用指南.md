# 1.   介绍
Intel® OpenVINO Toolkit是基于卷积神经网络（CNN）提供的一整套深度学习推断和神经网络优化的工具链和方法集，并提供了接口统一的跨Intel硬件及其加速器的异构计算框架，将DL算力水平发挥到最大，获得了业界认可。
ROS（机器人操作系统）是最流行的开源机器人软件框架，为机器人（特别是移动机器人）提供了一整套接口统一、分布式、功能齐全的全栈软硬件解决方案，拥有健全的生态系统和高度活跃的贡献者社区。
`ROS+OpenVINO`项目将OpenVINO工具套件移植到了ROS系统，不仅保留了OpenVINO工具套件灵活、高效、异构的深度学习算力水平和推断能力，也体现ROS系统的优势和特点，满足了针对ROS系统的开发、设计过程中对深度学习功能的需求：
- 标准的ROS通信接口（Topic，Service）
- 分布式模块化设计
- 基于配置文件的流水线化模块管理，轻松响应多变的场景、部署需求

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414162940342.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
目前已实现的主要功能包括：
- **支持多种数据来源**，例如普通USB摄像头、Intel Realsense深度摄像头、ROS/ROS2 Image topic、图片或视频文件等等。
- **支持多种常见的视觉推断功能**，例如人脸识别（表情、年龄、性别、头部朝向等等）、物体识别、基于交通场景的车辆、行人和道路识别以及物体分割等等功能。
- 推断结果的**多种结构化输出**，包括ROS2 Topic，OpenCV的image window，RViz机器人视觉呈现工具等等。

# 2.   先决条件
- 支持的CPU型号：x86_64，包括：
	- 6th-8th Generation       Intel® Core™
	- Intel® Xeon® v5 family
	-	Intel® Xeon® v6 family
- 支持的集成显卡配置（可选）
	- 6th to 8th generation       Intel® Core™ processors with Iris® Pro graphics and Intel® HD Graphics
	- 6th to 8th generation       Intel® Xeon® processors with Iris Pro graphics and Intel HD Graphics       (excluding the e5 product family, which does not have graphics)
	- Intel® Pentium®       processors N4200/5, N3350/5, N3450/5 with Intel HD Graphics
- 操作系统：Ubuntu 18.04 LTS
- ROS Melodic
- OpenVINO 2020.3 LTS
- RGB 摄像头（RealSense D400 系列或标准USB 摄像头）或 视频/图像文件作为输入

# 3.   环境搭建
- 安装Ubuntu18.04 LTS
- 安装ROS Melodic Desktop-Full：<https://wiki.ros.org/melodic/Installation/Ubuntu>， 建议参考文档《安装ROS环境时的常见问题及解决办法》
- 安装OpenVINO 2020.3 LTS：
	- 下载：<https://software.intel.com/content/www/us/en/develop/tools/openvino-toolkit/download.html>
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210414163449500.png)
	- 安装指南（需要完成Model Optimizer的配置）: <https://docs.openvinotoolkit.org/2020.3/_docs_install_guides_installing_openvino_linux.html>
	- 驱动：
		- 如果使用集成显卡，请参考以下部分安装驱动: <https://docs.openvinotoolkit.org/2020.3/_docs_install_guides_installing_openvino_linux.html#additional-GPU-steps>
		- 如果使用神经计算棒2，请参考以下部分安装驱动: <https://docs.openvinotoolkit.org/2020.3/_docs_install_guides_installing_openvino_linux.html#additional-NCS-steps>

	- 安装 Intel® RealSense™ SDK 2.0 (tag v2.17.1)
		-  [Install from source code (Recommended)](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/installation.md)
		-  [Install from package](https://github.com/IntelRealSense/librealsense/blob/v2.17.1/doc/distribution_linux.md)

	-  安装gflags依赖库
		```shell
		sudo apt-get install -y libgflags-dev
		```
		

# 4.   编译
-    获取代码
		```shell
		mkdir -p ~/catkin_ws/src
		cd ~/catkin_ws/src
		git clone https://github.com/intel/ros_openvino_toolkit -b dev-ov2020.3
		git clone https://github.com/intel/object_msgs
		```

- 编译
	```shell
	cd ~/catkin_ws
	catkin_make_isolated
	source ./ devel_isolated /setup.bash
	```

# 5.   运行Demo
## a. 准备模型
```shell
source /opt/intel/openvino/bin/setupvars.sh
sudo mkdir -p /opt/openvino_toolkit
sudo ln -s /opt/intel/openvino_2020.3.194/deployment_tools/open_model_zoo/tools/downloader /opt/openvino_toolkit/models
sudo chmod 777 -R /opt/openvino_toolkit/models
```
## b. 下载IR模型文件
```shell
cd /opt/openvino_toolkit/models/

sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-detection-adas-0001
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name age-gender-recognition-retail-0013
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name emotions-recognition-retail-0003
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name head-pose-estimation-adas-0001
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name person-detection-retail-0013
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name landmarks-regression-retail-0009
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name face-reidentification-retail-0095
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name vehicle-attributes-recognition-barrier-0039
sudo python3 /opt/intel/openvino/deployment_tools/tools/model_downloader/downloader.py --name license-plate-recognition-barrier-0001
```
## c. 下载public模型并转换为IR格式
### 1. vehicle-license-plate-detection-barrier-0123模型
```shell
cd /opt/openvino_toolkit/models/
sudo python3 ./downloader.py --name vehicle-license-plate-detection-barrier-0123
sudo python3 /opt/intel/openvino_2020.3.194/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=vehicle-license-plate-detection-barrier-0123 --mo /opt/intel/openvino_2020.3.194/deployment_tools/model_optimizer/mo.py 
```
### 2. mobilenet-ssd模型
```shell
cd /opt/openvino_toolkit/models/public
sudo git clone https://gitee.com/openvinotoolkit-prc/public_models.git
sudo mv public_models/mobilenet-ssd/ mobilenet-ssd/
cd ..
sudo python3 /opt/intel/openvino_2020.3.194/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=mobilenet-ssd --mo /opt/intel/openvino_2020.3.194/deployment_tools/model_optimizer/mo.py 
```
### 3. deeplabv3模型
```shell
cd /opt/openvino_toolkit/models/
sudo python3 ./downloader.py --name deeplabv3
sudo python3 /opt/intel/openvino_2020.3.194/deployment_tools/open_model_zoo/tools/downloader/converter.py --name=deeplabv3 --mo /opt/intel/openvino_2020.3.194/deployment_tools/model_optimizer/mo.py 
```


## d. 拷贝标签文件
```shell
cd /opt/openvino_toolkit/models
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels intel/emotions-recognition-retail-0003/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels intel/face-detection-adas-0001/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels public/vehicle-license-plate-detection-barrier-0123/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels public/mobilenet-ssd/FP16/
sudo cp ~/catkin_ws/src/ros_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels public/deeplabv3/FP16/
mv public/deeplabv3/FP16/frozen_inference_graph.labels  public/deeplabv3/FP16/deeplabv3.labels
```
## e.  运行Samples
```shell
source ~/catkin_ws/devel_isolated/setup.sh
roslaunch vino_launch pipeline_face_reidentification.launch
roslaunch vino_launch pipeline_image.launch
roslaunch vino_launch pipeline_object.launch
roslaunch vino_launch pipeline_people.launch
roslaunch vino_launch pipeline_reidentification.launch
roslaunch vino_launch pipeline_face_reidentification.launch
roslaunch vino_launch pipeline_segmentation.launch
roslaunch vino_launch pipeline_vehicle_detection.launch
roslaunch vino_launch pipeline_video.launch
```

# 