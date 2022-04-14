# basic_mobile_robot

https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/

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

### github請看這
https://cynthiachuang.github.io/Generating-a-Ssh-Key-and-Adding-It-to-the-Github/
# ssh-keygen -t ed25519 -C "smith.lai@gmail.com"
# vim ~/.ssh/config
```
# - github
Host            github.com
Hostname        github.com
# Port            222
User            smithlai
identityfile    ~/.ssh/githubsmith

```

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
# echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
# echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc
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
sudo apt install -y ros-foxy-robot-localization
```

### Other tools
```sh
sudo apt install ros-foxy-tf2-tools
sudo apt install evince

ros2 run tf2_tools view_frames  # galactic
#ros2 run tf2_tools view_frames.py  # foxy
evince frames.pdf

sudo apt install -y ros-foxy-rqt*
rqt-graph
```


---------------------------------

## Examples

### Example 0 (gazebo world))
> https://automaticaddison.com/how-to-load-a-world-file-into-gazebo-ros-2/  
> https://automaticaddison.com/useful-world-files-for-gazebo-and-ros-2-simulations/
> https://github.com/osrf/gazebo/tree/gazebo11/worlds  

```sh
mkdir  ~/dev_ws/src/basic_mobile_robot/models/cafe_world
cp -r ~/.gazebo/models/ground_plane ~/dev_ws/src/basic_mobile_robot/models/cafe_world
cp -r ~/.gazebo/models/cafe ~/dev_ws/src/basic_mobile_robot/models/cafe_world
cp -r ~/.gazebo/models/cafe_table ~/dev_ws/src/basic_mobile_robot/models/cafe_world
mkdir  ~/dev_ws/src/basic_mobile_robot/worlds/cafe_world
cp /usr/share/gazebo-11/worlds/cafe.world ~/dev_ws/src/basic_mobile_robot/worlds/cafe_world

ros2 launch two_wheeled_robot load_world_into_gazebo.launch.py
# gazebo_models_path = os.path.join(pkg_share, 'models', 'cafe_world')
# os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

```

### Example 1 (urdf, joint_state_publisher, robot_state_publisher)
key files:
> models/basic_mobile_bot_v1.urdf  
> rviz/urdf_config_v1.rviz  
> meshes/*  
> package.xml  
> CMakeLists.txt  
> launch/basic_mobile_bot_v1.launch.py  

```sh
ros2 launch basic_mobile_robot basic_mobile_bot_v1.launch.py
```

### Example2 (sdf, Gazebo’s IMU sensor plugin,  Gazebo’s differential drive plugin)
key files:
> models/basic_mobile_bot_description_v1/*  
> worlds/basic_mobile_bot_world_v1/smalltown.world  
> launch/basic_mobile_bot_v2.launch.py  

```sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/dev_ws/src/basic_mobile_robot/models/
```
```sh
killall gazebo; killall gzserver; killall gzclient
ros2 launch basic_mobile_robot basic_mobile_bot_v2.launch.py
ros2 run rqt_robot_steering rqt_robot_steering --force-discover

ros2 topic echo /wheel/odometry
ros2 topic info /imu/data
ros2 topic info /wheel/odometry
```

### Example3 ( robot_localization package, ekf.yaml )
key files:
> config/ekf_v1.yaml  
> package.xml  
> launch/basic_mobile_bot_v3.launch.py  


```sh
killall gazebo; killall gzserver; killall gzclient
ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py

# ros2 topic info /imu/data
# ros2 topic info /wheel/odometry
# ros2 topic echo /odometry/filtered
# ros2 node info /ekf_filter_node
# ros2 run tf2_ros tf2_echo odom base_footprint
# # ros2 run tf2_tools view_frames.py && evince frames.pdf  #foxy
ros2 run tf2_tools view_frames && evince frames.pdf

```

### Example4 ( lidar(libgazebo_ros_ray_sensor.so) )
key files:
> models/basic_mobile_bot_v2.urdf  
> models/basic_mobile_bot_description_v2/*  
> worlds/basic_mobile_bot_world_v2/smalltown.world  
> rviz/urdf_config_v2.rviz (urdf_config_v1 is also fine)  
> launch/basic_mobile_bot_v4.launch.py  


```sh
killall gazebo; killall gzserver; killall gzclient
ros2 launch basic_mobile_robot basic_mobile_bot_v4.launch.py

ros2 topic info /scan
```

### Example5 ( nav2 )
*Note*: Remember to set pose estimate in rviz2
key files:
> config/ekf_v2.yaml  
> params/nav2_params.yaml
> models/basic_mobile_bot_description_v3/*  
> worlds/basic_mobile_bot_world_v3/smalltown.world  
> rviz/nav2_config.rviz
> maps/*
> launch/basic_mobile_bot_v5.launch.py  

```sh
killall gazebo; killall gzserver; killall gzclient
ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py
# slam mode
ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py slam:=True
ros2 run rqt_robot_steering rqt_robot_steering --force-discover
# map server (this command is invalid in older foxy, refers to the tutorial)
ros2 run nav2_map_server map_saver_cli -f my_map
```