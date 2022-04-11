# basic_mobile_robot

## Install ROS2

### SSH (WSL)
sudo apt install ssh
`sudo vi /etc/ssh/sshd_config`
```
Port 2222
ChallengeResponseAuthentication yes
PasswordAuthentication yes
```

> sshd: no hostkeys available -- exiting.
`sudo dpkg-reconfigure openssh-server`


### Install ROS 2 Debian(Ubuntu 20)
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

#### ===== 1.locale
```sh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### ===== 2.Setup Sources
```sh
sudo apt update && sudo apt install -y curl gnupg2 lsb-release && \
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

#### add the repository to your sources list:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```


#### ===== 3.Install ROS 2 packages
```sh
sudo apt update
sudo apt install -y ros-foxy-desktop
#### sudo apt install ros-foxy-ros-base
sudo apt install -y python3-rosdep2


sudo rosdep init
rosdep update
# build前執行
# cd ~/ROS2_WS/dev_ws/
# rosdep install -i --from-path src --rosdistro foxy -y

#colcon
sudo apt install python3-colcon-common-extensions
```
#### ===== 4.Add sourcing to your shell startup script
> https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html
```sh
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
# Domain (0~232) https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html#domain-id-to-udp-port-calculator
# echo "export ROS_DOMAIN_ID=9" >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc
```


## Install Navigation2 and gazebo

```sh
sudo apt update
sudo apt-get -y install ros-foxy-gazebo-*

# Install Cartographer
# https://github.com/ros2/cartographer_ros
# This is hard to compile
# sudo apt install -y ros-foxy-cartographer
# sudo apt install -y ros-foxy-cartographer-ros
# Install Navigation2

sudo apt install -y ros-foxy-navigation2
sudo apt install -y ros-foxy-nav2-bringup
sudo apt install -y ros-foxy-joint-state-publisher-gui
sudo apt install -y ros-foxy-xacro
```

### Other tools
```sh
sudo apt install ros-foxy-tf2-tools
ros2 run tf2_tools view_frames.py
evince frames.pdf

sudo apt install -y ros-foxy-rqt*
rqt-graph
```


---------------------------------

## Examples

### Example 1
key files:
> models/basic_mobile_bot_v1/basic_mobile_bot_v1.urdf
> rviz/basic_mobile_bot_v1/urdf_config.rviz
> meshes/*
> package.xml
> CMakeLists.txt
> launch/basic_mobile_bot_v1.launch.py

```sh
ros2 launch basic_mobile_robot basic_mobile_bot_v1.launch.py
```

### Example2
key files:
> models/basic_mobile_bot_description_v1/*
> worlds/basic_mobile_bot_world/smalltown.world
> launch/basic_mobile_bot_v2.launch.py
> ros2 topic echo /wheel/odometry
```sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/dev_ws/src/basic_mobile_robot/models/
```
basic_mobile_bot_description_v1