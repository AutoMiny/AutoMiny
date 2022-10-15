---
title: "Installation"
permalink: /docs/installation/
excerpt: "Compile and install the autominy software on your computer"
toc: true
---
### Compile the autominy project
The autominy project must be installed on your computer to interact and develop for the car:

```bash
source /opt/ros/humble/setup.bash
sudo apt install git
git clone https://github.com/autominy/autominy
cd autominy/catkin_ws
sudo apt install python3-rosdep python3-colcon-common-extensions clang
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=humble -y
colcon build --symlink-install --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
source install/setup.bash
```
It is probably a good idea to add the project to the `.bashrc` (when using bash) or `.zshrc` (when using zsh).

When using bash:
```bash
echo "source /path/to/your/installation/catkin_ws/install/setup.bash" >> ~/.bashrc
```
When using zsh:
```zsh
echo "source /path/to/your/installation/catkin_ws/install/setup.zsh" >> ~/.zshrc
```

### Working on the car remotely
You can connect to the ROS running on the car remotely from your computer by setting the environment variable `ROS_DOMAIN_ID`:

The `ROS_DOMAIN_ID` must be the same on your computer as on the model car. The model car's 'ROS_DOMAIN_ID' can be calculated by `CAR_ID - 100`.

```bash
export ROS_DOMAIN_ID=[xyz]
```

It is a good idea to add these lines to your `.bashrc` or `.zshrc` file to have these variables set on every shell session automatically.