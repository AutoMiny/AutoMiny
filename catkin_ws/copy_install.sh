echo "replace root path"
pwd
cd ..
var=$(pwd)
echo "The catkin_ws directory $var."
sed -i 's#'$var'#/root#'g $var/catkin_ws/odroid-install/.catkin $var/catkin_ws/odroid-install/setup.sh $var/catkin_ws/odroid-install/_setup_util.py $var/catkin_ws/odroid-install/.rosinstall
sed -i 's#/opt/odroid-x2/sdk/#/#'g $var/catkin_ws/odroid-install/_setup_util.py
scp -r $var/catkin_ws/odroid-install root@192.168.43.102:./catkin_ws