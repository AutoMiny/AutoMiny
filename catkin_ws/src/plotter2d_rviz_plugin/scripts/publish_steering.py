#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from autominy_msgs.msg import Plot
from std_msgs.msg import Header

def main():
    pubWantedSteering = rospy.Publisher('/control/command/normalized_wanted_steering', Plot, queue_size=1)
    pubSteering = rospy.Publisher('/carstate/steering', Plot, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        time = rospy.Time.now()

        steeringCmd = Plot()
        steeringCmd.header = Header()
        steeringCmd.header.stamp = time
        steeringCmd.value = np.sin(time.to_sec())
        pubWantedSteering.publish(steeringCmd)

        steering = Plot()
        steering.header = Header()
        steering.header.stamp = time
        steering.value = np.cos(time.to_sec())
        pubSteering.publish(steering)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass