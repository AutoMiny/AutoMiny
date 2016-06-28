sudo scp -r model_car/catkin_ws root@192.168.1.199:./
sudo scp -r model_car/extra_library/glfw root@192.168.1.199:./
sudo scp -r model_car/extra_library/librealsense root@192.168.1.199:./
sudo scp model_car/autostart.sh root@192.168.1.199:.//
sudo scp model_car/.bashrc root@192.168.1.199:./
sudo scp set.sh root@192.168.1.199:./
sudo scp set2.sh root@192.168.1.199:./

echo "copy finished!"
