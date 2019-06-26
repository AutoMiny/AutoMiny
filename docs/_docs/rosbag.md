---
title: "Rosbag"
permalink: /docs/rosbag/
excerpt: "Recording and playing back data using rosbag"
toc: true
---

Sometimes it is easier to record data once and then play it back while developing for the car. We provide easy to use launch files to record and play back a rosbag. While playing back the whole AutoMiny software stack is launched manually so you have the same environment as running on a car.

### Recording
To start recording launch `Record.launch` from autominy:

```bash
roslaunch autominy Record.launch
```
This will capture any raw data from the sensors to a rosbag in the home folder. You can take a look at the captured topics in the `Record.launch` file.

### Playback
Copy the rosbag file from the car to your computer. Then run

```bash
roslaunch autominy Logged.launch logfile:=/ABSOLUTE/PATH/TO/BAGFILE.bag
```

This will launch the AutoMiny stack locally and play back the data that was recorded earlier. You can pause the rosbag at any time by pressing the space bar.