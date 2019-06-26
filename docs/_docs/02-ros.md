---
title: "Setting up ROS and the project"
permalink: /docs/ros/
excerpt: "Setting up the autominy project along with ROS"
toc: true
---
### Setting up ROS
These instructions assume that you are running Ubuntu 18.04. Install ROS
melodic using the installation guide at the
[ROS wiki](http://wiki.ros.org/melodic/Installation/Ubuntu). At the end of that tutorial you will have a working ROS installation onto which the autominy packages will be installed.

### Compile the autominy project
The autominy project must be installed on your computer to interact and develop for the car:

```bash
git clone https://github.com/autominy/autominy
cd autominy/catkin_ws
apt install python-catkin-tools
rosdep install --from-paths . --ignore-src --rosdistro=melodic -y
catkin build
source devel/setup.bash # if using bash
source devel/setup.zsh # if using zsh
```
It is probably a good idea to add the project to the `.bashrc` (when using bash) or `.zshrc` (when using zsh).

When using bash:
```bash
echo "source /path/to/your/installation/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
When using zsh:
```zsh
echo "source /path/to/your/installation/catkin_ws/devel/setup.zsh" >> ~/.zshrc
```


### Working on the car remotely
You can connect to the ROS running on the car remotely from your computer by setting the environment variables `ROS_MASTER_URI` and `ROS_IP`:

```bash
export ROS_MASTER_URI="http://192.168.43.[NUMBER]:11311/"
export ROS_IP="[YOUR COMPUTER'S IP]"
```

It is a good idea to add these lines to your `.bashrc` or `.zshrc` file to have these variables set on every shell session automatically.
