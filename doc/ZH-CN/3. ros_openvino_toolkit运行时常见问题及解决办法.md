# ros_openvino_toolkit运行时常见问题指南
## a. 电脑上安装了多个版本的OpenCV，导致ros_openvino_toolkit运行不起来。
**解决办法**：修改ROS cv_bridge

 1. 背景
由于OpenVINO 2020.3的OpenCV版本是OpenCV4.3.0， ROS melodic默认的OpenCV是OpenCV3.2，如果使用者还安装了其它OpenCV版本，很容易导致OpenCV指向的路径出问题，导致项目无法运行，因此需要对cv_bridge做本地更新。

 1. 主要修改内容
	a. cv_bridge CMakeLists.txt添加对OpenCV4版本的判断。
	b. 在`cv_bridge/src/module_opencv3.cpp`的基础之上，更新两个函数，变成`cv_bridge/src/module_opencv4.cpp`

 3. 具体修改步骤
	1. 从github下载`ROS cv_bridge`，并切换到ROS melodic版本
		```shell
		git clone https://github.com/ros-perception/vision_opencv.git
		git checkout melodic
		```
	2. 修改`cv_bridge/src/CMakeLists.txt`文件:
		```shell
		# 在头部添加
		set (CMAKE_CXX_STANDARD 11)
		
		# 从第34行到 target_link_libraries之前的endif() ，全部换成
		if (OpenCV_VERSION_MAJOR VERSION_EQUAL 2)
		add_library(${PROJECT_NAME}_boost module.cpp module_opencv2.cpp)
		elseif(OpenCV_VERSION_MAJOR VERSION_EQUAL 3)
		add_library(${PROJECT_NAME}_boost module.cpp module_opencv3.cpp)
		elseif(OpenCV_VERSION_MAJOR VERSION_EQUAL 4)
		add_library(${PROJECT_NAME}_boost module.cpp module_opencv4.cpp)
		endif()
		```
	3. 复制`cv_bridge/src/module_opencv3.cpp`为`cv_bridge/src/module_opencv4.cpp`,并对module_opencv4.cpp中的两个函数进行修改
		```cpp
		// 修改前的
		UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const
		to
		// 修改后的
		UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, AccessFlag flags, UMatUsageFlags usageFlags) const
		
		// 修改前的
		bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const
		to
		// 修改后的
		bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags) const
		```

 4. 编译安装，检查环境
修改好后，在工作空间内运行`catkin_make`，运行`roscd cv_bridge && pwd`，检查`cv_bridge`路径是否为本地安装路径。
	> ps: 笔者在此处曾经栽过跟头，当时编译安装后，继续编译ros_openvino_toolkit工程，结果链接的库还是opencv 3.2.0。
	> 后来直接将`/opt/ros/melodic/share/`目录下的`cv_bridge`名称换成`cv_bridge_back`，这时候系统只能找到本地的`cv_bridge`.不过我觉得一定还有更好的办法。
	> 如果各位读者有何更好的建议，欢迎与我联系，同时我也将这部分内容进行更新！

	 参考链接:
	1.  <https://github.com/ros-perception/vision_opencv/issues/272>
	2. <https://github.com/ros-perception/vision_opencv/pull/259>
	3.  <https://github.com/ros-perception/vision_opencv/commit/1f2ef094016897373d6bcf5607c19579ee39cc33>
	4. <https://stackoverflow.com/questions/58978543/how-to-link-opencv-4-with-ros-cv-bridge>

## b. protobuf需要更新
如果在运行mo时报错:
```shell
[ ERROR ]  
Detected not satisfied dependencies:
    protobuf: not installed, required: 3.6.1
```
根据提示，这是由于protobuf版本太低导致，因此需要对其进行更新。

**解决办法：更新protobuf**
1. protobuf下载列表: <https://github.com/protocolbuffers/protobuf/releases>
2. 解压安装
```shell
tar -xf protobuf-all-3.6.1.tar.gz
cd protobuf-3.6.1
./configure
make
make check
sudo make install
cd ./python
python3 setup.py build
python3 setup.py test
sudo python3 setup.py install
```
- 参考链接: <https://community.intel.com/t5/Intel-Distribution-of-OpenVINO/protobuf-not-installed-required-3-6-1/m-p/1154807/highlight/true?profile.language=zh-TW>
