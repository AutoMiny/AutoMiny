---
title: "CLion"
permalink: /docs/clion/
excerpt: "Setting up CLion"
toc: true
---
CLion is the recommended IDE to develop with. CLion features support for both C++ and Python and integrates with the CMake system used by ROS. Jetbrains offers free CLion licenses for students for educational purposes on their website.

### ROS CMake behaviour
CLion needs the project's ROS environment variables. You can make sure that CLion has all required environment variables by sourcing the project's `setup.bash`/`setup.zsh` in your `.bashrc`/`.zshrc`. Then run CLion from a terminal (running from the toolbox or app launcher will not source the ROS environment!). You also need to have build the project at least once before running CLion. CLion uses plain CMake whilst ROS uses catkin to interface with CMake. Catkin sets `CMAKE_INSTALL_PREFIX` and `CATKIN_DEVEL_PREFIX` to specify the build and install folders for the common ROS workspace layout. The following script functions as a intermediate script between CLion and CMake and sets these variables so that artifacts end up in the expected folders. Paste the following script into a file called `cmake-wrapper.sh` and make it executable. In CLion, select `File` -> `Settings` and go to `Build, Execution, Deployment` and select `Toolchains`. Select the cmake-wrapper script as CMake executable.

```bash
#!/bin/bash
CATKIN_PARAMS=""
# Check if CMAKE_BUILD_TYPE is present (only used when loading the project)
if [[ $1 == *"CMAKE_BUILD_TYPE"* ]]; then
	CATKIN_PARAMS="-DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel"
fi

cmake $CATKIN_PARAMS "$@"
```

### Symbolic linking
By default workspaces are configured to use a linked development workspace. The symbolic linking is not handled by CMake but by catkin after a package has been build. Since again CLion is not using catkin but interfacing directly with CMake, this step is not performed when building through CMake. Fortunately, this behaviour can be changed to a merged development workspace which is not symbolic linked. In a terminal use the following command inside your `catkin_ws` folder to disable symbolic linking:
```bash
catkin config --merge-devel
```

### Opening the project
In CLion select `Open Project` from the file menu. Navigate to the autominy project and select `CMakeLists.txt` from `catkin_ws/src/` and confirm to open as project.

### Recommended plugins
The [hatchery](https://github.com/duckietown/hatchery) plugin is highly recommended which can be installed using the plugin browser. It features basic integration with ROS such as roslaunch, syntax highlighting for launch files, URDF and much more.

### Use clang by default
If you want to leverage clang's faster compilation speed and better error messages, clang can be configured as the default toolchain in a catkin workspace. Open up a terminal in your catkin workspace and type:
```bash
catkin config --cmake-args -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++
```

You can also enable Debug mode to add debug symbols:
```bash
catkin config --cmake-args -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_BUILD_TYPE=Debug
```