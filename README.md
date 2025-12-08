# Introduction

This project establishes a complete intelligent driving experimental platform, built on ROS Noetic and Ubuntu 20.04, primarily designed to validate a mobile robot’s capabilities in perception, mapping, localization, and path planning within real-world environments.

In this tutorial, you will learn how to use the ISSPA source code and how PA (Pyhsical Agents) are launched! Specifically, we will guide you on how to setup your system with respect to the following topics:


- `Developing Environment`

- `WorkSpace Setup`

Developing Environment
----------------------

We recommend using **Ubuntu 20.04** as your operating system for development, since it is the last version 
that support ROS1 noetic, the version of the Robot Operating System we use at the moment; 
we will upgrade to ROS2 in future.

From now on, we assume you have already installed these components in your machine:

- `Ubuntu 20.04 <https://releases.ubuntu.com/20.04/>`_

- `ROS1 noetic <http://wiki.ros.org/noetic>`_

The following are some of the dependency libraries that you need to install before ISSPA can be compiled and run:

    sudo apt update
    sudo apt install libuvc-dev libgoogle-glog-dev ros-noetic-costmap-2d ros-noetic-nav-core libceres-dev

You might also need to install ``git`` to clone the `ISSPA <https://github.com/iscas-tis/ISS-PA/>`_ repository:


    sudo apt install git



WorkSpace Setup
----------------------
ISSPA is our main repository; it is recommended that you first understand its directory structure, which is necessary for later use and development.

First, open a terminal and clone the source code inside your preferred directory; we just use the /home/$USER/ directory as an example:
 ```
  cd /home/$USER
  git clone https://github.com/AholicJH/ISSPA.git

 ```
Next, use catkin_make to compile the workspace.

 ```
 cd /home/$USER/ISSPA
 catkin_make
 ```
Use
----------------------
1 . Firstly, launch chassis and sensors driver and sensors of the vehicle.

 ```
 roslaunch pavs_bringup pavs_chassis_and_sensors.launch
 ```
With the chassis booted, you can view the current list of messages via the rostopic list, e.g. /cmd_vel is the topic for which the chassis expects twist subscribers.

At this point, launch another terminal, again using SSH to connect to the vehicle, and enter the following commands to test that the motors and servos are working properly.
 ```
 rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10

 ```
 If the chassis was successfully activated, the vehicle should have moved forward and turn left by now.

 2 . After that, you can test if the SLAM program works properly.

```
roslaunch mapping_baselines pavs_map.launch
```

When the program is started, you can check for message output by typing rostopic echo /map in the vehicle’s terminal, which normally outputs a number of matrices containing decimal values between 0 and 1, which represent the probability of an obstacle being present in the grid.

The default SLAM algorithm is gmapping and you can conveniently switch between algorithms by passing arguments on the command line: for example, if you want to use cartographer, you can use the following command:


```
roslaunch mapping_baselines pavs_map.launch map_type:=cartographer
```
Further, you need to control the vehicle movement via a remote controller or a keyboard control node. When the map is created, you can execute 
`map.sh` under the `~/ISSPA/src/isspa_mapping/mapping_baselines/scripts` directory to save your map.

A quick way to search for this script is roscd, used as follows:
```
roscd mapping_baselines/scripts
```

Then execute the following command:
```
./map.sh
# If you are using `cartographer`, use `cartographer_map.sh`
./cartographer_map.sh
```
Eventually, the maps will be saved to the `~/ISSPA/src/isspa_mapping/mapping_baselines/maps/` directory with the name map.
At this point, you will find the following two files:

```
Chongqing@SWU:~/ISSPA/src/isspa_mapping/mapping_baselines/maps$ ll
total 640
drwxrwxr-x 2 chw chw   4096 1月  16 20:46 ./
drwxrwxr-x 6 chw chw   4096 12月 19 16:37 ../
-rw-rw-r-- 1 chw chw 640052 1月  16 20:46 map.pgm
-rw-rw-r-- 1 chw chw    191 1月  16 20:46 map.yaml
```
where `map.pgm` is the grip map and `map.yaml` is the configuration file for the map.

3 . Start the Navigation Program

Once you have activated the vehicle’s chassis and sensors, and you have obtained a grid map, it is then possible to realize the task of fixed-point navigation!

```
roslaunch navigation_stack pavs_navigation.launch
```
By now, you will be able to test the effectiveness of the navigation algorithms on RVIZ by selecting points on the map that are free of obstacles.