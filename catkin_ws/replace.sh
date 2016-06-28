echo "replace root path"
pwd
cd ..
var=$(pwd)
echo "The catkin_ws directory $var."
sed -i 's#$var#/root#'g $var/catkin_ws/odroid-devel/.catkin $var/catkin_ws/odroid-devel/setup.sh $var/catkin_ws/odroid-devel/.rosinstall $var/catkin_ws/odroid-devel/_setup_util.py
sed -i 's#/opt/odroid-x2/sdk/#/#'g $var/catkin_ws/odroid-devel/_setup_util.py
scp -r $var/catkin_ws/src root@192.168.1.199:./catkin_ws
scp -r $var/catkin_ws/odroid-* root@192.168.1.199:./catkin_ws
echo "finished"
