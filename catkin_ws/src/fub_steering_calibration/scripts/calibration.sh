for i in 950 2150
do
	echo "Put the car infront of a wall in around 1 meter distance
     ___
    |	\   ||
    |___/   ||
     car    wall
	"
echo "Press any key to continue... "
read t
echo "Desired command angle= " $i
python angle_calibrator_online.py $i

done
sed -i '1i<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n<!DOCTYPE boost_serialization>' SteerAngleActuator.xml

CAR_IP=`echo $ROS_MASTER_URI | awk '{split($0,a,":"); print a[2]}' | awk '{split($0,a,"//"); print a[2]}'`
echo "CAR_IP is" $CAR_IP
scp SteerAngleActuator.xml root@$CAR_IP:/opt/model_car/catkin_ws/src/odometry/cfg/
