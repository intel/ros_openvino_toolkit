#!/bin/bash -x

set -euxo pipefail

if [[ -n "$1" && -n "$2" ]]; then
	HOST_NAME=$1
	ROOT_PASSWD=$2
	echo "set sudo password to $ROOT_PASSWD and your hostname is $HOST_NAME"
else
        echo "you have to input your hostname and sudo password!"
        echo "    for example:./environment_setup.sh username password"
	exit
fi

basedir=$PWD
echo "Begin Environment Setup"

system_ver=`cat /etc/lsb-release | grep -i "DISTRIB_RELEASE" | cut -d "=" -f2`

#Get Config Parameters
CLEAN=`cat modules.conf | grep 'clean'`
CLEAN=${CLEAN##*=}
echo "Set CLEAN to $CLEAN"

ROS_DEBIAN=`cat modules.conf | grep 'ros_debian'`
ROS_DEBIAN=${ROS_DEBIAN##*=}
echo "Set ROS_DEBIAN to $ROS_DEBIAN"

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
  echo "===================Cleaning...===================================="
  rm -rf ~/code
  #echo $ROOT_PASSWD | sudo -S apt-get purge -y ros-kinetic-*
  echo $ROOT_PASSWD | sudo -S rm -rf /opt/intel
  rm -rf ~/Downloads/l_openvino_toolkit*
  if [[ $system_ver = "16.04" && -L "/usr/lib/x86_64-linux-gnu/libboost_python3.so" ]]; then
    echo $ROOT_PASSWD | sudo -S rm /usr/lib/x86_64-linux-gnu/libboost_python3.so
  fi
fi

# Setup ROS from debian
if [ "$ROS_DEBIAN" == "1" ]; then
  echo "===================Installing ROS from Debian Package...======================="
  echo $ROOT_PASSWD | sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

  echo $ROOT_PASSWD | sudo -S apt-get update
  echo $ROOT_PASSWD | sudo -S apt-get install -y ros-kinetic-desktop-full

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
  tail ~/.bashrc | grep "/opt/ros/kinetic/setup.bash"
  set -o errexit 

  if [ "$?" == "1" ]; then
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
  else
    echo "ros kinetic already set, skip..."
  fi
  source ~/.bashrc
  echo $ROOT_PASSWD | sudo -S apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
fi

#setup OPENVINO
if [ "$OPENVINO" == "1" ]; then
  cd ~/Downloads
  wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/13521/l_openvino_toolkit_p_2018.3.343.tgz
  tar -xvf l_openvino_toolkit_p_2018.3.343.tgz
  cd l_openvino_toolkit_p_2018.3.343
  echo $ROOT_PASSWD | sudo -S ./install_cv_sdk_dependencies.sh
  cp $basedir/openvino_silent.cfg .
  echo $ROOT_PASSWD | sudo -S ./install.sh --silent openvino_silent.cfg

  set +o errexit 
  tail ~/.bashrc | grep "computer_vision_sdk/bin/setupvars.sh"
  set -o errexit
 
  if [ "$?" == "1" ]; then
    echo "source /opt/intel/computer_vision_sdk/bin/setupvars.sh" >> ~/.bashrc
  else
    echo "openvino already set, skip..."
  fi
fi

#install OPENCL Driver for GPU
if [ "$OPENCL" == "1" ]; then
   cd /opt/intel/computer_vision_sdk/install_dependencies
   echo $ROOT_PASSWD | sudo -S ./install_NEO_OCL_driver.sh
   echo "install OPENCL Driver for GPU"
fi

# Setup LIBREALSENSE
if [ "$LIBREALSENSE" == "1" ]; then
  echo "===================Setting Up LibRealSense...======================="
  echo $ROOT_PASSWD | sudo -S apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  echo $ROOT_PASSWD | sudo -S apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  mkdir -p ~/code && cd ~/code
  git clone https://github.com/IntelRealSense/librealsense
  cd ~/code/librealsense
  git checkout v2.14.1
  mkdir build && cd build
  cmake ../ -DBUILD_EXAMPLES=true
  echo $ROOT_PASSWD | sudo -S make uninstall
  make clean
  make
  echo $ROOT_PASSWD | sudo -S make install

  cd ..
  echo $ROOT_PASSWD | sudo -S cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  echo $ROOT_PASSWD | sudo -S udevadm control --reload-rules
  udevadm trigger
  echo "==== END install librealsense ===="
fi

# Setup other dependencies
if [ "$OTHER_DEPENDENCY" == "1" ]; then
  echo "===================Setting UP OTHER_DEPENDENCY DEPENDENCY...======================="
  echo $ROOT_PASSWD | sudo -S apt-get install python3-pip
  pip3 install numpy
  if [ $system_ver = "16.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     cd /usr/lib/x86_64-linux-gnu
     sudo ln -s libboost_python-py35.so libboost_python3.so
  elif [ $system_ver = "18.04" ]; then
     echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
     sudo apt install libboost-python1.62.0
   fi
   echo "==== END install other dependencies ===="
fi

echo "Environment setup successfully"
