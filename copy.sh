ls ~/.ssh/id_rsa.pub || ssh-keygen
ls ~/.ssh/id_rsa.pub || ssh-keygen
ssh-copy-id root@192.168.1.199
scp -r catkin_ws root@192.168.1.199:./
scp -r extra_library/glfw root@192.168.1.199:./
scp -r extra_library/librealsense root@192.168.1.199:./
scp autostart.sh root@192.168.1.199:./
scp .bashrc root@192.168.1.199:./
scp installation_file/set.sh root@192.168.1.199:./
scp extra_library/72-serial.rules root@192.168.1.199:/etc/udev/rules.d/
scp extra_library/71-usb-cam.rules root@192.168.1.199:/etc/udev/rules.d/
echo "copy finished!"


