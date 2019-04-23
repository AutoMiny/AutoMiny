The realsense camera is interfaced using librealsense which consists of a driver library and kernel patches. Unfortunately, on arm there are no prebuild packages for librealsense so we will have to build the library along with the kernel patches ourselves. The following script will install librealsense and the realsense ros node.

# Updating Realsense (lib and ROS node)
```
pkill ros
apt update
apt upgrade -y
reboot

pkill ros
cd /opt/librealsense
git clean -fdx
git fetch
git checkout v2.21.0
./scripts/patch-realsense-ubuntu-odroid-xu4-4.14.sh
rm -rf build
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release
make uninstall && make clean && make -j1 && make install

rm -rf /opt/model_car/catkin_ws/src/realsense
mkdir -p /opt/realsense-ros/src/
cd /opt/realsense-ros/src/
git clone https://github.com/intel-ros/realsense
cd ../
catkin config --cmake-args "-DCMAKE_BUILD_TYPE=Release"
catkin build -j2
```