#!/bin/sh

# Install dependencies
sudo apt-get install ros-melodic-moveit* ros-melodic-serial ros-melodic-joint-state-publisher-gui -y

# Installing third party packages
cd src/
wstool init .
wstool merge art-med_robots_ros/dependencies.rosinstall
wstool update   
