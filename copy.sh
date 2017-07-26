scp -r catkin_ws root@192.168.1.199:./
scp autostart.sh root@192.168.1.199:./
scp .bashrc root@192.168.1.199:./
scp 72-serial.rules root@192.168.1.199:/etc/udev/rules.d/
scp 71-usb-cam.rules root@192.168.1.199:/etc/udev/rules.d/
echo "copy finished!"


