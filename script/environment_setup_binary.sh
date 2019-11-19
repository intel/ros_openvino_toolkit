#!/bin/bash

set -euo pipefail

echo "Please Enter Your Password:"
stty -echo
read ROOT_PASSWD
stty echo

basedir=$PWD
echo "Begin Environment Setup"

system_ver=`cat /etc/lsb-release | grep -i "DISTRIB_RELEASE" | cut -d "=" -f2`

#Ubuntu >=18.04 not support ros kinetic
if [ $system_ver = "16.04" ]; then
  ros_ver="kinetic"
  echo "Found Ubuntu $system_ver, install ros-$ros_ver"
elif [ $system_ver = "18.04" ]; then 
  ros_ver="melodic"
  echo "Found Ubuntu $system_ver, install ros-$ros_ver"
else 
  ros_ver="melodic"
  echo "Found not official support Ubuntu $system_ver, currently support 16.04 and 18.04"
fi

#Get Config Parameters
CLEAN=`cat modules.conf | grep 'clean'`
CLEAN=${CLEAN##*=}
echo "Set CLEAN to $CLEAN"

ROS_DEBIAN=`cat modules.conf | grep 'ros_debian'`
ROS_DEBIAN=${ROS_DEBIAN##*=}
echo "Set ROS_DEBIAN to $ROS_DEBIAN"

OPENCV=`cat modules.conf | grep 'opencv'`
OPENCV=${OPENCV##*=}
echo "Set OPENCV to $OPENCV"

OPENVINO=`cat modules.conf | grep 'openvino'`
OPENVINO=${OPENVINO##*=}
echo "Set OPENVINO to $OPENVINO"

OPENCL=`cat modules.conf | grep 'opencl'`
OPENCL=${OPENCL##*=}
echo "Set OPENCL to $OPENCL"

LIBREALSENSE=`cat modules.conf | grep 'librealsense'`
LIBREALSENSE=${LIBREALSENSE##*=}
echo "Set LIBREALSENSE to $LIBREALSENSE"

OTHER_DEPENDENCY=`cat modules.conf | grep 'other_dependency'`
OTHER_DEPENDENCY=${OTHER_DEPENDENCY##*=}
echo "Set OTHER_DEPENDENCY to $OTHER_DEPENDENCY"


# Clean Existing Directories
if [ "$CLEAN" == "1" ]; then
  echo "===================Clean Existing Directories...===================================="

  read -n1 -p "The clean operation will delete some manually created directories,
  including ~/code, /opt/intel, /opt/openvino_toolkit, and OpenVINO tar ball.
  Do you want to clean existing directories[Y/N]?" answer
  case $answer in
        Y|y) echo
                echo "===================Cleaning...===================================="
  		echo $ROOT_PASSWD | sudo -S rm -rf ~/code
  		echo $ROOT_PASSWD | sudo -S rm -rf /opt/intel
  		echo $ROOT_PASSWD | sudo -S rm -rf /opt/openvino_toolkit
  		if [[ $system_ver = "16.04" && -L "/usr/lib/x86_64-linux-gnu/libboost_python3.so" ]]; then
    			echo $ROOT_PASSWD | sudo -S rm /usr/lib/x86_64-linux-gnu/libboost_python3.so
  		fi
                echo "===================Clean finish...====================================";;
        N|n) echo
                echo "===================not clean, continue...====================================";;
  esac
fi

# Setup ROS from debian
if [ "$ROS_DEBIAN" == "1" ]; then
  echo "===================Installing ROS from Debian Package...======================="
  echo $ROOT_PASSWD | sudo -S apt-get install -y curl
  curl http://repo.ros2.org/repos.key | sudo apt-key add -
  echo $ROOT_PASSWD | sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --keyserver-options http-proxy="$http_proxy" --recv-key F42ED6FBAB17C654
  #echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver 'hkp://pgp.mit.edu:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  #For those who cannot access hkp protocal
  echo $ROOT_PASSWD | sudo -S apt-get update
  echo $ROOT_PASSWD | sudo -S apt-get install -y ros-$ros_ver-desktop-full

  if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo $ROOT_PASSWD | sudo -S rosdep init
  else
    echo "file already exists, skip..."
  fi

  set +o errexit 
  rosdep update
  until [ $? == 0 ]
  do
    rosdep update
  done
  tail ~/.bashrc | grep "/opt/ros/$ros_ver/setup.bash"
  set -o errexit 

  if [ "$?" == "1" ]; then
    echo "source /opt/ros/$ros_ver/setup.bash" >> ~/.bashrc
  else
    echo "ros $ros_ver already set, skip..."
  fi
  source ~/.bashrc
  echo $ROOT_PASSWD | sudo -S apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
fi

# Setup OpenCV
if [ "$OPENCV" == "1" ]; then
  echo "===================Installing OpenCV3 from Source...======================="
  echo $ROOT_PASSWD | sudo -S apt-get install -y build-essential
  echo $ROOT_PASSWD | sudo -S apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  echo $ROOT_PASSWD | sudo -S apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libpng-dev libtiff-dev libdc1394-22-dev

  if [ $system_ver = "18.04" ]; then
    echo $ROOT_PASSWD | sudo -S add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
    echo $ROOT_PASSWD | sudo apt update
    echo $ROOT_PASSWD | sudo apt install libjasper1 libjasper-dev
  else
    echo $ROOT_PASSWD | sudo -S apt-get install libjasper-dev
  fi

  mkdir -p ~/code && cd ~/code
  echo "begin clone opencv"
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git
  echo "finish clone opencv"

  cd ~/code/opencv
  git checkout 3.4.2
  cd ~/code/opencv_contrib
  git checkout 3.4.2

  cd ~/code/opencv
  mkdir build && cd build
  cmake -DOPENCV_EXTRA_MODULES_PATH=$HOME/code/opencv_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j4
  echo $ROOT_PASSWD | sudo -S make install
  echo $ROOT_PASSWD | sudo -S ldconfig
  echo "==== END install OpenCV ===="
fi

#setup OPENVINO
if [ "$OPENVINO" == "1" ]; then
  cd ~/Downloads
  wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/16057/l_openvino_toolkit_p_2019.3.376.tgz
  tar -xvf l_openvino_toolkit_p_2019.3.376.tgz
  cd l_openvino_toolkit_p_2019.3.376
  echo $ROOT_PASSWD | sudo -S ./install_openvino_dependencies.sh
  cp $basedir/openvino_silent.cfg .
  echo $ROOT_PASSWD | sudo -S ./install.sh --silent openvino_silent.cfg

  set +o errexit 
  tail ~/.bashrc | grep "openvino/bin/setupvars.sh"
  set -o errexit
 
  if [ "$?" == "1" ]; then
    echo "source /opt/intel/openvino/bin/setupvars.sh" >> ~/.bashrc
  else
    echo "openvino already set, skip..."
  fi
fi

#install OPENCL Driver for GPU
if [ "$OPENCL" == "1" ]; then
   cd /opt/intel/openvino/install_dependencies
   echo $ROOT_PASSWD | sudo -S ./install_NEO_OCL_driver.sh
   echo "install OPENCL Driver for GPU"
fi

# Setup LIBREALSENSE
if [ "$LIBREALSENSE" == "1" ]; then
  echo "===================Setting Up LibRealSense...======================="

  echo "Install server public key for librealsense"
  if [ -n "$http_proxy" ]; then
    echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --keyserver-options http-proxy=$http_proxy --recv-key C8B3A55A6F3EFCDE
  else
    echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
  fi
  if ! test "$(grep "http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo" /etc/apt/sources.list)"
  then
    echo $ROOT_PASSWD | sudo -S add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
  fi

  echo $ROOT_PASSWD | sudo -S apt update && sudo -S apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
  echo "==== END install librealsense ===="
fi

# Setup other dependencies
if [ "$OTHER_DEPENDENCY" == "1" ]; then
  echo "===================Setting UP OTHER_DEPENDENCY DEPENDENCY...======================="
  echo $ROOT_PASSWD | sudo -S apt-get install -y python3-pip
  sudo pip3 install numpy
  sudo pip3 install networkx
  if [ $system_ver = "16.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     cd /usr/lib/x86_64-linux-gnu
     echo $ROOT_PASSWD | sudo -S ln -sf libboost_python-py35.so libboost_python3.so
  elif [ $system_ver = "18.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     echo $ROOT_PASSWD | sudo -S apt install -y --no-install-recommends libboost-python1.62.0
   fi
   echo "==== END install other dependencies ===="
fi

echo "Environment setup successfully"
