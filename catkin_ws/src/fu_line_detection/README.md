kinect
======

Stuff to get the "kinect/ASUS" data published in ros, and maybe do stuff with it.

Installation
============

What was previously known as `openni_kinect` is now split into two packages: 1.[openni-camera](http://www.ros.org/wiki/openni_camera) 2.[openni-launch](http://www.ros.org/wiki/openni_launch).
Install them:

```bash
$ sudo apt-get install ros-groovy-openni-camera ros-groovy-openni-launch
```

Then, you can run the process which will publish the camera's data. (Of course, `roscore` should be running.)

```bash
$ roslaunch openni_launch openni.launch
```

Viewing the topics
==================

Check which topics are published by running `rostopic list` and, more interestingly, look at the streams using

```bash
$ rosrun image_view image_view image:=/camera/rgb/image_color
$ rosrun image_view image_view image:=/camera/depth/image
```

Troubleshooting
===============

USB interface is not supported
------------------------------

Asus Xtion PRO live model 601 which is mounted on METRA robots fails to work with ROS. The [solution we found on the internet](http://answers.ros.org/question/61211/problem-with-xtion-pro-live-and-openni_camera/) is the following:

    Edit the file: /etc/openni/Globaldefauts.ini (make a backup first)

    find the line UsbInterface and uncomment it to force it to use 'BULK' endpoints: UsbInterface=2

We tried this already with one of the robot and it works.


Failed to set USB interface
---------------------------

This error may occur when executing the `roslaunch openni...` command.

The ASUS Xtion has [firmware problems with USB 3.0 interfaces](http://reconstructme.net/2012/10/13/asus-xtion-usb-3-0-hotfix/).
Befor trying to patch the firmware (on Windows), just plug it into some other USB interface.
Modern laptops have some USB 3.0 and some USB 2.0 interfaces, so you might be lucky to hit a 2.0 one which just works.

Another possible solution, reported to work for some, is to load the following kernel modules:

```bash
$ sudo modprobe -r gspca_kinect
$ sudo modprobe -r gspca_main
```


Creating a C++ node listening to the Kinect
===========================================
Let's get going!
I'll use `$WORKSPACE` to be your workspace folder.

Creating a new project in ROS
-----------------------------

### Disclaimer
I don't know all of the ROS terminologies yet, I may use wrong words for things.

### Creating a package
First, creating a package:

```bash
$ cd $WORKSPACE/src
$ catkin_create_pkg vision roscpp
$ cd $WORKSPACE
$ catkin_make
```

There can be many "nodes" (apps/mains/...) in one package.
A node may be written in C++ or Python.

### Creating a C++ node
Write your code. Minimal code, in a file we'll call `test_ir.cpp` just for giggles, and place it into the `$WORKSPACE/src/vision` folder:

```cpp
#include <iostream>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Foo_bar_baz_quux");

    std::cout << "Oh hai there!" << std::endl;

    return 0;
}
```

Add this thingy to the `CMakeLists.txt` file of your package, so that `catkin_make` knows what to do.
The previous step involving `catkin_create_pkg` did create a template `CMakeLists.txt` file in `$WORKSPACE/src/vision`
which you can have a look at and uncomment the things you want. For our minimal project that
results in:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED roscpp)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_executable(test_ir_node test_ir.cpp)

target_link_libraries(test_ir_node
  ${catkin_LIBRARIES}
)

install(TARGETS test_ir_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Then, you need to compile your package by running `catkin_make`. But be wary, you'll get a (descriptive) error
message if you run it in the wrong folder. Run it from `$WORKSPACE`.
You might need to `source $WORKSPACE/devel/setup.bash` before proceeding.

If it all succeeds, you can run the `test_ir_node` executable/node in the `vision` package by then typing:

```bash
$ rosrun vision test_ir_node
```

It should greet you warmly.

Face Detection using cob_perception 
===================================

1. `git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git` and do `rosmake`
2. `git clone https://github.com/ipa320/cob_perception_common.git` and check out the groovy branch `git checkout groovy_dev` and do `rosmake`
3. `git clone https://github.com/ipa320/cob_people_perception.git` and check out the groovy branch `git checkout groovy_dev` and do `rosmake`
4. use `roslaunch openni_launch openni.launch` to start kinect
5. `roslaunch cob_people_detection people_detection.launch`

Visualizing a color point cloud of scene using ROS & RVIZ
=========================================================

1. `roslaunch openni_launch openni.launch`
2. `rosrun rqt_reconfigure rqt_reconfigure` and select /camera/driver from the drop-down menu. Enable the depth_registration checkbox. 
3. `rosrun rviz rviz`
4.  Set the Fixed Frame (top left of rviz window) to camera_depth_optical_frame.  
5.  Add a PointCloud2 display and set PointCloud2 topic to /camera/depth_registered/points. Set Color Transformer to RGB8. You should see a color, 3D point cloud of your scene. 
