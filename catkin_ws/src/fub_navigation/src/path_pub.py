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


class PathReader:
    def __init__(self, trajectory_map):
        # get the list of poses defined in the RNDF file
        self.poses = read_map(trajectory_map)

        # Also publish a Path data structure to visualize the path in rviz
        self.path_publisher = rospy.Publisher("/assignment10/path", Path, queue_size=1, latch=True)

        # set up the Path data structure
        self.path = Path()
        self.path.header = Header()
        self.path.header.frame_id = 'map'
        self.path.poses = [PoseStamped(pose=x, header=Header()) for x in self.poses]

    def publish(self):
        """Just update timestamp and publish the fixed data structure."""
        self.path.header.stamp = rospy.Time.now()
        self.path_publisher.publish(self.path)
        print("path is published")


def main():
    rospy.init_node('assignment10_trajectory')
    path_reader = PathReader('sample_map_origin_map.txt')  # constructor creates publishers / subscribers
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        path_reader.publish()
        rate.sleep()

if __name__ == '__main__':
    main()

