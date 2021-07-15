import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
import lanelet2
from lanelet2.core import Lanelet, BasicPoint2d
from lanelet2.geometry import findNearest, toArcCoordinates, to2D, length
from lanelet2.routing import RoutingGraph
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
import random

class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")
        projector = UtmProjector(Origin(52, 13, 0))
        self.map = load("mapfile.osm", projector)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = RoutingGraph(self.map, traffic_rules)
        self.lane_pub = rospy.Publisher("/navigation/lane", Int64, queue_size=1)
        self.odom_sub = rospy.Subscriber("/simulation/odom_ground_truth", Odometry, self.on_odom, queue_size=1)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal, queue_size=1)
        self.goal = None
        self.path = None
        self.odom = None

    def on_goal(self, goal):
        goal: PoseStamped
        x = goal.pose.position.x
        y = goal.pose.position.y
        pos = BasicPoint2d(x, y)
        self.goal = findNearest(self.map.laneletLayer, pos, 1)[0][1]

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        pos = BasicPoint2d(x, y)
        start = findNearest(self.map.laneletLayer, pos, 1)[0][1]

        route = self.graph.getRoute(start, self.goal)
        self.path = list(route.shortestPath())
        self.path = list(filter(lambda l: "intersection" not in l.attributes, self.path))

    def on_odom(self, odom):
        self.odom = odom

        if self.path is None and self.path != []:
            return

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        pos = BasicPoint2d(x, y)
        closest_lane = findNearest(self.map.laneletLayer, pos, 1)[0][1]
        if "intersection" in closest_lane.attributes and closest_lane.attributes["intersection"] == "yes":
            return

        lane_length = length(to2D(closest_lane.centerline))
        arc_coordinates = toArcCoordinates(to2D(closest_lane.centerline), pos)

        # we can start transition
        if closest_lane == self.path[0] and arc_coordinates.length > lane_length - 0.8:
            self.path.pop(0)

        # we reached the goal
        if not self.path:
            self.path = None
            return

        next_lane = self.path[0]

        self.lane_pub.publish(next_lane.id)

if __name__ == "__main__":
    StateMachine()
    rospy.spin()