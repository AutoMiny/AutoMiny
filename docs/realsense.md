## Realsense driver
The realsense camera is interfaced using librealsense which consists of a driver library and kernel patches.


### Updating Realsense (Intel NUC)
On x64 we just use the provided packages from Intel. librealsense is already provided by the packages along with the needed kernel patches.

```
sudo apt update
sudo apt upgrade -y
cd /opt/autominy/catkin_ws/realsense-ros
git pull
catkin build
```

### Updating Realsense (Odroid XU4)
Unfortunately, on ARM (Odroid XU4) there are no prebuild packages for librealsense so we will have to build the library along with the kernel patches ourselves. The following script will install librealsense and the realsense ros node.

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

cd /opt/autominy/catkin_ws/src/realsense-ros
git pull
catkin build -j1
```