#!/bin/bash
set -euo pipefail

# PASSWD
echo "Please Enter Your Password:"
stty -echo
read ROOT_PASSWD
stty echo
basedir=$PWD

#system_version
system_ver=`cat /etc/lsb-release | grep -i "DISTRIB_RELEASE" | cut -d "=" -f2`

#Show Select Box Windows
resize -s 24 80
#terminator --geometry=485x299 -b
SELECT=$(whiptail --title "Ubuntu $system_ver ros_openvino_toolkit env setup" --checklist \
"space to select, ESC to back, Enter to execute" 24 80 16 \
"CLEAN_ALL" "Clean all folders" OFF \
"ROS_ENV" "Instal ROS-desk-full from Debian Package" OFF \
"OpenCV" "Setup OpenCV" OFF \
"OpenCL" "Setup OpenCL" OFF \
"DLDT" "Setup DLDT" OFF \
"MODEL_ZOO" "Setup open_model_zoo" OFF \
"LIBREALSENSE" "Setup LIBREALSENSE" OFF \
"OTHERS" "Setup other dependencies" OFF \
"HELP" "Show help" OFF \
3>&1 1>&2 2>&3
)

# Functions
function clean {
  echo "   ======       Setting UP clean..    ======    "
  read -n1 -p "The clean operation will delete some manually created directories,
  including ~/code, /opt/intel, /opt/openvino_toolkit, and OpenVINO tar ball.
  Do you want to clean existing directories[Y/N]?" answer
  case $answer in
        Y|y) echo
                echo "   ======    Cleaning...   ======    "
  		echo $ROOT_PASSWD | sudo -S rm -rf ~/code
  		echo $ROOT_PASSWD | sudo -S rm -rf /opt/intel
  		echo $ROOT_PASSWD | sudo -S rm -rf /opt/openvino_toolkit
  		if [[ $system_ver = "16.04" && -L "/usr/lib/x86_64-linux-gnu/libboost_python3.so" ]]; then
    			echo $ROOT_PASSWD | sudo -S rm /usr/lib/x86_64-linux-gnu/libboost_python3.so
  		fi
                echo "   ======    Clean finish...   ======    ";;
        N|n) echo
                echo "   ======    not clean, continue...   ======    ";;
  esac
}

function ros {
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

  echo "   ======    Installing ROS from Debian Package...   ======    "
  echo $ROOT_PASSWD | sudo -S sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  #echo $ROOT_PASSWD | sudo -S apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
  #For those who cannot access hkp protocal 
  echo $ROOT_PASSWD | curl http://repo.ros2.org/repos.key | sudo apt-key add -
  echo $ROOT_PASSWD | sudo -S apt-get update
  echo $ROOT_PASSWD | sudo -S apt-get install -y ros-$ros_ver-desktop-full #For ubuntu16.04 Ros-melodic

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
    echo "ros melodic already set, skip..."
  fi
  source ~/.bashrc
  echo $ROOT_PASSWD | sudo -S apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

function opencv {
  echo "   ======    Installing OpenCV3 from Source...   ======    "
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
  git clone https://gitee.com/cxs5534/opencv.git
  git clone https://gitee.com/cxs5534/opencv_contrib.git
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
}

function opencl {
  echo "   ======    Installing OpenCL Driver for GPU...   ======    "

  mkdir -p ~/code && cd ~/code
  wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
  unzip SRB5.0_linux64.zip -d SRB5.0_linux64
  cd SRB5.0_linux64
  echo $ROOT_PASSWD | sudo -S apt-get install xz-utils
  mkdir intel-opencl
  tar -C intel-opencl -Jxf intel-opencl-r5.0-63503.x86_64.tar.xz
  tar -C intel-opencl -Jxf intel-opencl-devel-r5.0-63503.x86_64.tar.xz
  tar -C intel-opencl -Jxf intel-opencl-cpu-r5.0-63503.x86_64.tar.xz
  echo $ROOT_PASSWD | sudo -S cp -R intel-opencl/* /
  echo $ROOT_PASSWD | sudo -S ldconfig
  echo "==== END install OpenCL ===="
}

function dldt {
  echo "   ======    Installing Deep Learning Deployment Toolkit...   ======    "

  if [[ -f /etc/lsb-release ]]; then
    sudo -E apt update
    sudo -E apt-get install -y \
            build-essential \
            cmake \
            curl \
            wget \
            libssl-dev \
            ca-certificates \
            git \
            libboost-regex-dev \
            gcc-multilib\
            libgtk2.0-dev \
            pkg-config \
            unzip \
            automake \
            libtool \
            autoconf \
            libcairo2-dev \
            libpango1.0-dev \
            libglib2.0-dev \
            libgtk2.0-dev \
            libswscale-dev \
            libavcodec-dev \
            libavformat-dev \
            libgstreamer1.0-0 \
            gstreamer1.0-plugins-base \
            libusb-1.0-0-dev \
            libopenblas-dev
    if [ $system_ver = "18.04" ]; then
            sudo -E apt-get install -y libpng-dev
    else
            sudo -E apt-get install -y libpng12-dev
    fi
  fi
  mkdir -p  ~/code && cd ~/code
  git clone https://gitee.com/cxs5534/dldt.git
  cd dldt/inference-engine/
  git checkout 2018_R5 
  git submodule init
  git submodule update --recursive
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j8
  echo $ROOT_PASSWD | sudo -S mkdir -p /opt/openvino_toolkit
  echo $ROOT_PASSWD | sudo -S ln -s ~/code/dldt /opt/openvino_toolkit/dldt
  echo "==== END install DLDT ===="
}

function module_zoo {
  echo "   ======    Installing Open Model Zoo...   ======    "
  mkdir -p ~/code && cd ~/code
  git clone https://gitee.com/cxs5534/open_model_zoo
  cd open_model_zoo/demos/
  git checkout 2018_R5  
  mkdir build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release /opt/openvino_toolkit/dldt/inference-engine
  make -j8
  echo $ROOT_PASSWD | sudo -S mkdir -p /opt/openvino_toolkit
  echo $ROOT_PASSWD | sudo -S ln -s ~/code/open_model_zoo /opt/openvino_toolkit/open_model_zoo
  echo "==== END install open_model_zoo ===="
}

function librealsense {
  echo "   ======    Setting Up LibRealSense...   ======    "
  echo $ROOT_PASSWD | sudo -S apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
  echo $ROOT_PASSWD | sudo -S apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  #mkdir -p ~/code && cd ~/code
  #git clone https://gitee.com/cxs5534/librealsense.git
  cd ~/code/librealsense
  git checkout v2.17.1
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
}

function others {
  echo "   ======    Setting UP OTHER_DEPENDENCY DEPENDENCY...   ======    "
  pip3 install numpy
  pip3 install networkx
  echo $ROOT_PASSWD | sudo -S apt-get install python3-yaml
  if [ $system_ver = "16.04" ]; then
    echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
    cd /usr/lib/x86_64-linux-gnu
    sudo ln -s libboost_python-py35.so libboost_python3.so
  elif [ $system_ver = "18.04" ]; then
    echo $ROOT_PASSWD | sudo -S apt-get install -y --no-install-recommends libboost-all-dev
    sudo apt install libboost-python1.62.0
  fi
  echo "==== END install other dependencies ===="
}


function HELP {
  echo "   ======     help...   ======    "
  echo "Contents need to be added"
  echo "Please try again"
  exit
}

existstatus=$?
if [ $existstatus = 0 ]; then
  echo $SELECT | grep "HELP" && HELP
  echo $SELECT | grep "CLEAN" && clean
  echo $SELECT | grep "ROS_ENV" && ros
  echo $SELECT | grep "OpenCV" && opencv
  echo $SELECT | grep "OpenCL" && opencl
  echo $SELECT | grep "DLDT" && dldt
  echo $SELECT | grep "MODEL_ZOO" && module_zoo
  echo $SELECT | grep "LIBREALSENSE" && librealsense
  echo $SELECT | grep "OTHERS" && others
else
  echo "EXIT"
  break
fi

