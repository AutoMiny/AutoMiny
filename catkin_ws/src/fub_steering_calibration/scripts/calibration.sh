for i in 0 30 60 90 120 150 180
do
	echo "Put the car infront of a wall in around 1 meter distance
     ___
    |	\   ||
    |___/   ||
     car    wall
	"
read -p "Press any key to continue... " -n1 -s
echo
echo "Desired command angle= " $i
python angle_calibrator_online.py $i

done

scp SteerAngleActuator.xml root@192.168.43.111:./catkin_ws/src/odometry/cfg/
