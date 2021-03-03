# Gazebo Jackal Ultra-wideband Environment

<p align="center">
    <a href="https://github.com/Marius-Juston/UWB-Jackal-World/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/Marius-Juston/UWB-Jackal-World" /></a>
    <a href="https://github.com/Marius-Juston/UWB-Jackal-World/pulse" alt="Activity">
        <img alt="GitHub commit activity" src="https://img.shields.io/github/commit-activity/m/Marius-Juston/UWB-Jackal-World"></a>
    <a href="https://github.com/Marius-Juston/UWB-Jackal-World/stargazers">
        <img alt="Stars" src="https://img.shields.io/github/stars/Marius-Juston/UWB-Jackal-World"></a>
    <a href="https://github.com/Marius-Juston/UWB-Jackal-World/network/members">
        <img alt="Forks" src="https://img.shields.io/github/forks/Marius-Juston/UWB-Jackal-World"></a>
    <a href="https://github.com/Marius-Juston/UWB-Jackal-World/issues">
        <img alt="Issues" src="https://img.shields.io/github/issues/Marius-Juston/UWB-Jackal-World"></a>
    <a href="./LICENSE" alt="Activity">
        <img alt="GitHub" src="https://img.shields.io/github/license/Marius-Juston/UWB-Jackal-World"></a>
</p>

## 1. Table of contents
- [Gazebo Jackal Ultra-wideband Environment](#gazebo-jackal-ultra-wideband-environment)
  - [1. Table of contents](#1-table-of-contents)
  - [2. Project Description](#2-project-description)
    - [2.1. The robot](#21-the-robot)
    - [2.2. The world](#22-the-world)
  - [3. Installation](#3-installation)
  - [4. Launch](#4-launch)
    - [4.1. Jackal Only](#41-jackal-only)
    - [4.2. Stationary world](#42-stationary-world)


## 2. Project Description

This project is meant to simulate an environment where a [Jackal](https://clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/) is combined with [Decawave Ultra-wideband](https://www.decawave.com/product/mdek1001-deployment-kit/) sensors to improve localization. (This is in [Gazebo](http://gazebosim.org/) )

### 2.1. The robot

![Image of Jackal robot setup](/images/real-robot.jpg)
![Simulated Jackal robot](/images/simulated-robot.jpg)

This is the Jackal from Clearpath Robotics, on its sides (labeled in red circles) are two Decawave Ultra-wideband sensors setup as tags.

### 2.2. The world

![Simulated world](images/default_gzclient_camera(1)-2021-03-02T23_27_44.583334.jpg)

This is currently the only simulated world. There are currently 4 UWB sensors setup as anchors which send their range measurement.

## 3. Installation

This project uses [`ROS Melodic`](http://wiki.ros.org/melodic) as its ROS backend.

To setup:

1. Either create a catkin workspace or navigate to the `src` folder
2. ```git clone git@github.com:Marius-Juston/UWB-Jackal-World.git```
3. `sudo apt-get install ros-melodic-jackal-simulator ros-melodic-jackal-desktop ros-melodic-jackal-navigation`
   - This will install all the necessary melodic packages for the Jackal as well as its base model
   - For more detailed instuctions on the Jackal API: https://www.clearpathrobotics.com/assets/guides/melodic/jackal/simulation.html
4. Create a `gtec` folder in your `catkin_ws/src` folder
5. Navigate to your `catkin_ws/src/gtec` folder
6. `git clone https://github.com/valentinbarral/gazebosensorplugins.git`
   - This installs the UWB plugin library which allows for the tags and anchors to send values
7. `git clone https://github.com/valentinbarral/rosmsgs`
   - This installs the custom UWB ranging messages
8. Navigate to your base `cakin_ws` folder
9.  `rospack profile`
10. `catkin_make`
11. `source ~/catkin_ws/devel/setup.bash`
   -  This step is very important. Be sure to run this every time you open up a new terminal. If you do not you will get errors when trying to run the world, that you cannot see certain packages.
   -  To make things easier if you only have a single ROS build and  `catkin_ws`, then you can run: 
   - `echo "source ~/catkin_ws/devel/setup.bash" > ~/.bashrc`
   -  This will allow you to not have to run the source command every time.


## 4. Launch

### 4.1. Jackal Only

To launch a Gazebo world with only a Jackal:

```bash

roslaunch jackal_world empty_world.launch
```

### 4.2. Stationary world

To launch a Gazebo world with the Jackal and the stationary anchors:


```bash

roslaunch jackal_world gazebo_world_launcher.launch
```
