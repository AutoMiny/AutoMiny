scp -r model_car/catkin_ws root@192.168.1.199:./
scp -r model_car/extra_library/glfw root@192.168.1.199:./
scp -r model_car/extra_library/librealsense root@192.168.1.199:./
scp model_car/autostart.sh root@192.168.1.199:.//
scp model_car/.bashrc root@192.168.1.199:./
scp set.sh root@192.168.1.199:./
scp set2.sh root@192.168.1.199:./

echo "copy finished!"
