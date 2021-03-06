# ART-Med Robots ROS Packages
*Pioneer LX with two JACO2 arms*

## Requirements

- Ubuntu 16.04
- ROS Kinetic

- Install [MoveIt](http://wiki.ros.org/moveit)
  ```
  sudo apt-get install ros-kinetic-moveit*
  ```

- Install [Kinova ROS](https://github.com/Kinovarobotics/kinova-ros)
  ```
  cd catkin_ws/src
  git clone https://github.com/Kinovarobotics/kinova-ros.git
  ```
  
- Install [AMR Robots](https://github.com/MobileRobots/amr-ros-config)
  ```
  cd catkin/src
  git clone https://github.com/MobileRobots/amr-ros-config.git
  ```
  
- Install [FLIR PTU ROS driver](https://github.com/ros-drivers/flir_ptu)
  ```
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/flir_ptu.git
  ```

- Install dependencies
  ```
  sudo apt-get install ros-kinetic-serial
  sudo apt-get install ros-kinetic-joint-state-publisher-gui
  ```

- Install Gazebo 9 with ROS Kinetic
  ```
  sudo apt-get remove ros-kinetic-gazebo* gazebo*
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*
  sudo apt dist-upgrade
  ```

## Preparation

```
cd catkin_ws/src
git clone https://github.com/juyoun726/art-med_robots_ros.git
cd ..
catkin_make
```

## How to Run

- Rviz
  ```
  roslaunch art_med_robots_description robot_rviz.launch
  ```
  <img src="https://github.com/juyoun726/art-med_robots_ros/blob/master/images/robot_rviz.png" width="640">
  
- Gazebo world
  ```
  roslaunch art_med_robots_gazebo robot_world.launch
  ```
  <img src="https://github.com/juyoun726/art-med_robots_ros/blob/master/images/robot_world.png" width="640">
  
- Gazebo & ROS control
  ```
  roslaunch art_med_robots_gazebo robot_world.launch
  roslaunch art_med_robots_control robot_control.launch
  ```
  or
  ```
  roslaunch art_med_robots_gazebo robot_gazebo.launch
  ```
 
- Example of moving a joint
  ```
  rostopic pub -1 /robot/left_joint_2_position_controller/command std_msgs/Float64 "data: 2.0"
  ```
  <img src="https://github.com/juyoun726/art-med_robots_ros/blob/master/images/robot_gazebo.png" width="640">
 
- Controlling the FLIR PTU unit
  ```
  roslaunch art_med_robots_gazebo robot_sensors_gazebo.launch
  rostopic pub -1 /robot/ptu_pan_position_controller/command std_msgs/Float64 "data: 5.0"
  rostopic pub -1 /robot/ptu_tilt_position_controller/command std_msgs/Float64 "data: -0.8"
  ```
  <img src="https://github.com/juyoun726/art-med_robots_ros/blob/master/images/pan.png" width="320">
  <img src="https://github.com/juyoun726/art-med_robots_ros/blob/master/images/tilt.png" width="320">

 
