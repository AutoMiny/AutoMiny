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
sed -i '1i<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n<!DOCTYPE boost_serialization>' SteerAngleActuator.xml

CAR_IP=`echo $ROS_MASTER_URI | awk '{split($0,a,":"); print a[2]}' | awk '{split($0,a,"//"); print a[2]}'`
echo "CAR_IP is" $CAR_IP
scp SteerAngleActuator.xml root@$CAR_IP:/opt/ros/modelcar/catkin_ws/src/odometry/cfg/
scp SteerAngleActuator.xml root@$CAR_IP:/opt/ros/modelcar/catkin_ws/install/share/odometry/cfg
