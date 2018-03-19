for i in 0 30 60 90 120 150 180
do
	echo "Put the car infront of a wall
     ___
    |	\   ||
    |___/   ||
     car    wall
	"
read -p "Press any key to continue... " -n1 -s
echo
echo "Desired command angle= " $i
python drive_angle.py $i

done
