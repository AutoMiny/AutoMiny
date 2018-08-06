#!/usr/bin/env python2

import rospy

from path_parser import read_points

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path

def read_map(map_file):
    """
    Read the file with the map data and return a Trajectory.
    :return: List of Pose elements
    """
    points = []

    for x, y in read_points(map_file):
        xy_point = Point(x, y, 0)
        xy_pose  = Pose(position=xy_point)
        points.append(xy_pose)

    return points


class loopReader:
    def __init__(self, point_map,name):
        # get the list of poses defined in the RNDF file
        self.poses = read_map(point_map)

        # Also publish a loop data structure to visualize the loop in rviz
        self.loop_publisher = rospy.Publisher(name, Path, queue_size=1, latch=True)

        # set up the path data structure
        self.loop = Path()
        self.loop.header = Header()
        self.loop.header.frame_id = 'map'
        self.loop.poses = [PoseStamped(pose=x, header=Header()) for x in self.poses]

    def publish(self):
        """Just update timestamp and publish the fixed data structure."""
        self.loop.header.stamp = rospy.Time.now()
        self.loop_publisher.publish(self.loop)
        print("loop is published")


def main():
    rospy.init_node('assignment10_trajectory')
    loop_inner = loopReader('new_map_inner_loop.txt',"inner")  # constructor creates publishers / subscribers
    loop_inner.publish()
    loop_outer = loopReader('new_map_outer_loop.txt',"outer")  # constructor creates publishers / subscribers
    loop_outer.publish()
    loop_middle= loopReader('new_map_middle_loop.txt',"middle")  # constructor creates publishers / subscribers
    loop_middle.publish()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        loop_inner.publish()
        loop_outer.publish()
        loop_middle.publish()
        rate.sleep()

if __name__ == '__main__':
    main()

