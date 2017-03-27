#!/bin/bash

echo "Warning: Make sure you have a kernel >=4.4 with a patched uvcvideo module installed (see https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cp config/99-realsense-libusb.rules /etc/udev/rules.d/
mkdir -p /usr/local/include/librealsense
cp -r include/librealsense/* /usr/local/include/librealsense
mkdir -p /usr/local/lib/arm-linux-gnueabihf
cp -r lib/* /usr/local/lib/arm-linux-gnueabihf
ldconfig

echo "All files installed!"
