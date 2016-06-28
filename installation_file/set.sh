cd glfw
make install
cd ..
cd librealsense
cp config/99-realsense-libusb.rules /etc/udev/rules.d/
cp config/uvc.conf /etc/modprobe.d/

echo "set 1 is finished, reboot please! "

