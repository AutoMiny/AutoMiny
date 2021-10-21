---
title: "Installation"
permalink: /docs/installation/
excerpt: "Compile and install the autominy software on your computer"
toc: true
---
### Compile the autominy project
The autominy project must be installed on your computer to interact and develop for the car:

```bash
git clone https://github.com/autominy/autominy
cd autominy/catkin_ws
apt install python3-catkin-tools python3-osrf-pycommon
rosdep install --from-paths . --ignore-src --rosdistro=noetic -y
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

### Speeding up compilation using CLang
You can use Clang instead of the (default) gcc compiler. Clang is much faster during compilation and uses less memory. The following commands install clang and tell catkin to use the clang compiler by default.

```bash
sudo apt install clang
catkin config --cmake-args -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++
```

### Working on the car remotely
You can connect to the ROS running on the car remotely from your computer by setting the environment variables `ROS_MASTER_URI` and `ROS_IP`:

```bash
export ROS_MASTER_URI="http://192.168.43.[NUMBER]:11311/"
export ROS_IP="[YOUR COMPUTER'S IP]"
```

It is a good idea to add these lines to your `.bashrc` or `.zshrc` file to have these variables set on every shell session automatically.