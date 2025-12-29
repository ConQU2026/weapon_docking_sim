# 依赖安装

```bash
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-dev \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-msgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  libgoogle-glog-dev \
  libpcl-dev \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  libeigen3-dev \
```


# 运行仿真
先colcon build然后source balabala的
And then

```bash
ros2 launch weapon_dock weapon_dock_sim.launch.py
```



## 其他的包的安装路径
```bash
git clone git@github.com:ConQU2026/auto_serial_bridge.git
git clone git@github.com:stm32f303ret6/livox_laser_simulation_RO2.git
git clone git@github.com:Ericsii/FAST_LIO_ROS2.git
```


# Fast-Lio2 测试, 效果还行
![test](./assets/fast-lio-test.png)
