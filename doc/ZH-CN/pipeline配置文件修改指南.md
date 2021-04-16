# ros_openvino_toolkit pipeline配置文件修改指南

这里以 [pipeline_people.yaml](https://github.com/intel/ros_openvino_toolkit/blob/dev-ov2020.3/vino_launch/param/pipeline_people.yaml)为示例:
```yaml
Pipelines:
- name: people
  inputs: [StandardCamera]
  infers:
    - name: FaceDetection
      model: /opt/openvino_toolkit/models/intel/face-detection-adas-0001/FP16/face-detection-adas-0001.xml
      engine: CPU
      label: /to/be/set/xxx.labels
      batch: 1
      confidence_threshold: 0.5
      enable_roi_constraint: true # set enable_roi_constraint to false if you don't want to make the inferred ROI (region of interest) constrained into the camera frame
    - name: AgeGenderRecognition
      model: /opt/openvino_toolkit/models/intel/age-gender-recognition-retail-0013/FP16/age-gender-recognition-retail-0013.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
    - name: EmotionRecognition
      model: /opt/openvino_toolkit/models/intel/emotions-recognition-retail-0003/FP16/emotions-recognition-retail-0003.xml
      engine: CPU
      label: be/set/xxx.labels
      batch: 16
    - name: HeadPoseEstimation
      model: /opt/openvino_toolkit/models/intel/head-pose-estimation-adas-0001/FP16/head-pose-estimation-adas-0001.xml
      engine: CPU
      label: to/be/set/xxx.labels
      batch: 16
  outputs: [ImageWindow, RosTopic, RViz]
  connects:
    - left: StandardCamera
      right: [FaceDetection]
    - left: FaceDetection
      right: [AgeGenderRecognition, EmotionRecognition, HeadPoseEstimation, ImageWindow, RosTopic, RViz]
    - left: AgeGenderRecognition
      right: [ImageWindow, RosTopic, RViz]
    - left: EmotionRecognition
      right: [ImageWindow, RosTopic, RViz]
    - left: HeadPoseEstimation
      right: [ImageWindow, RosTopic, RViz]

Common:
```

- **name**: 这里指代第2行的内容`- name: people`，表示pipeline的名称，可以用任何非空的字符串代替。在ROS Topic输出时作为命名空间一样的存在，比如我将pipeline name设置为`AAAAA`，对应的ROS Topic就包含`/openvino_toolkit/AAAAA/headposes`。 这个参数在多个pipeline同时运行的场景下使用，参考`multi_pipleine_service_object.yaml`文件。

- **inputs**: input相关的参数只能有一个<br>

|option|Description|
|--------------------|------------------------------------------------------------------|
|StandardCamera|任何支持USB输出的RGB摄像头. 现阶段只能识别`/dev/video0`|
|RealSenseCamera| Intel RealSense RGB-D 摄像头, 通过openCV的librealsense插件直接调用RealSense.|
|RealSenseCameraTopic| 任何ROS image消息结构的ROS Topic，参考命令`rosmsg show sensor_msgs/Image`|
|Image| 任何能被OpenCV处理的图片文件, 例如 .`png`, `.jpeg`.|
|Video| 任何能被OpenCV处理的视频.|

- **input_path** : 当输入源为`Image`或者`Video`时，需要指定的对应文件绝对路径。

- **infers**: `Inference Engine`是一组C++类，在`ros_openvino_toolkit`中提供API来读取IR文件，设置输入和输出格式，并完成设备上加载和执行模型的操作。

- **name**: Inference engine的名称. 现阶段支持的pipeline以及相关infers可以参考对应表格 [TODO]。

- **model**: IR模型的绝对路径. 

- **engine**: 现阶段支持`CPU`,`GPU`,`FPGA`,`MYRIAD`</br>


- **label**: OpenVINO识别结果对应于真实世界中的名称，一般设置成`to/be/set/xxx.labels`即可。注意yaml文件中指定的IR模型路径下，需要存放和模型名相同的`labels`文件(具体有哪些模型需要labels可参考表格)


- **batch**
在inference engine网络中动态批处理任务的大小。(Enable dynamic batch size for inference engine net. )

- **outputs**: 输出参数可以设置一个或者多个，现阶段的Output可以有以下选项:

|option|Description|
|--------------------|------------------------------------------------------------------|
|ImageWindow| 输出结果显示在ImageWindow中 |
|RosTopic| OpenVINO发布的推理的结果封装成ROS Topic|
|RViz| 将输出结果在Rviz中显示 |
| RosService | ROS Service信息 |

- **confidence_threshold**: 检测可置信度相关阈值。

- **connects**: pipeline的拓扑结构(树形结构)。 在yaml文件中，一个`left`只能设定一个值, 而`right`可以设定多个值。比如例子中的`FaceDetection`对应的`right`就包含`AgeGenderRecognition`,`EmotionRecognition`,`HeadPoseEstimation`,`ImageWindow`

