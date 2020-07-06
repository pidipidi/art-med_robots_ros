# ART-Med Robots ROS Packages

## Requirements

- Ubuntu 16.04
- ROS Kinetic
- [Kinova ROS](https://github.com/Kinovarobotics/kinova-ros)
  ```
  cd catkin_ws/src
  git clone https://github.com/Kinovarobotics/kinova-ros.git
  ```
  
- [AMR Robots](https://github.com/MobileRobots/amr-ros-config)
  ```
  cd catkin/src
  git clone https://github.com/MobileRobots/amr-ros-config.git
  ```
  
- [FLIR PTU ROS driver](https://github.com/ros-drivers/flir_ptu)
  ```
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/flir_ptu.git
  ```

- Dependencies
  ```
  sudo apt-get install ros-kinetic-serial
  sudo apt-get install ros-kinetic-joint-state-publisher-gui
  ```

## Preparation

```
cd catkin_ws/src
git clone https://github.com/juyoun726/art-med_robots_ros.git
cd ..
catkin_make
```

## How to Run

- Pioneer LX with two JACO2 arms
  ```
  cd catkin_ws/src/art-med_robots_ros/launch
  roslaunch pioneer-lx-jaco2-joint-state-gui.launch
  ```
