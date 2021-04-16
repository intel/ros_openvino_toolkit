# 安装ROS环境
## a. 换源
```shell
# 备份原来的sources.list
sudo cp /etc/apt/sources.list /etc/apt/sources.list_backup
# 修改sources.list
sudo gedit /etc/apt/sources.list
```
将`sources.list`替换成以下内容: 
```shell
# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse

# 预发布软件源，不建议启用
# deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
```

## b. 添加ros-latest.list并更新
参考ROS wiki官方教程安装时，有可能会出现如图所示的错误，这时可以尝试以下解决办法:
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210410181043538.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
```shell
sudo sh -c 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ bionic main
" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

> 参考链接: 
> 1. Ubuntu 镜像使用帮助- 清华大学开源软件镜像站: <https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/>
> 2. ROS Wiki:Ubuntu install of ROS Melodic: <http://wiki.ros.org/melodic/Installation/Ubuntu>
> 3. ROS 镜像使用帮助-清华大学开源软件镜像站: <https://mirrors.tuna.tsinghua.edu.cn/help/ros/>
--- 
## c. 激活ros环境
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## d. 安装ROS编译相关的包
```shell
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
按照官网教程，运行以下命令:
```shell
sudo rosdep init
sudo rosdep fix-permissions
rosdep update
```
这时候报错：cannot download default sources list
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210410213947221.png)
解决办法: 
>进入网站<https://ip.tool.chinaz.com/raw.githubusercontent.com>，我们可以看到`raw.githubusercontent.com` 的IPV4地址，将其添加到`/etc/hosts`中，如
>```shell
>185.199.109.133              raw.githubusercontent.com
>```
>并尝试运行`sudo rosdep init`
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210410215739613.png?x-oss-process=image,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2N4czU1MzQ=,size_16,color_FFFFFF,t_70)
>如果运行`rosdep init`命令且终端显示超时报错时，可以尝试以下办法:
>```shell
>sudo gedit /etc/resolv.conf
>```
>在文件末尾添加以下内容:
>```shell
>nameserver 8.8.8.8 #google域名服务器
>nameserver 8.8.4.4 #google域名服务器
>```

PS: 为了加速GitHub访问，推荐将以下内容添加到 `/etc/hosts` 文件中:

```shell
# GitHub Host Start

185.199.108.154              github.githubassets.com
140.82.113.22                central.github.com
185.199.108.133              desktop.githubusercontent.com
185.199.108.153              assets-cdn.github.com
185.199.108.133              camo.githubusercontent.com
185.199.108.133              github.map.fastly.net
199.232.69.194               github.global.ssl.fastly.net
140.82.114.3                 gist.github.com
140.82.114.5                 api.github.com
185.199.108.133              raw.githubusercontent.com
185.199.108.133              user-images.githubusercontent.com
185.199.108.133              favicons.githubusercontent.com
185.199.108.133              avatars5.githubusercontent.com
185.199.108.133              avatars4.githubusercontent.com
185.199.108.133              avatars3.githubusercontent.com
185.199.108.133              avatars2.githubusercontent.com
185.199.108.133              avatars1.githubusercontent.com
185.199.108.133              avatars0.githubusercontent.com
185.199.108.133              avatars.githubusercontent.com
140.82.113.9                 codeload.github.com
52.216.160.43                github-cloud.s3.amazonaws.com
52.216.179.211               github-com.s3.amazonaws.com
52.217.69.236                github-production-release-asset-2e65be.s3.amazonaws.com
52.216.185.107               github-production-user-asset-6210df.s3.amazonaws.com
52.217.13.92                 github-production-repository-file-5c1aeb.s3.amazonaws.com
185.199.108.133              media.githubusercontent.com

# Please Star : https://github.com/ineo6/hosts
# Mirror Repo : https://gitee.com/ineo6/hosts
# Update at: 2021-03-29 18:16:31

# GitHub Host End
```

