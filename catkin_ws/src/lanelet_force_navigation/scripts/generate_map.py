import rospy
import lanelet2
import os
import numpy as np
from collections import defaultdict

from lanelet2.io import Origin, write, load
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
from lanelet2.projection import UtmProjector


class TwoLaneStraight:
    def __init__(self, start_right: np.ndarray, start_left: np.ndarray, end_right: np.ndarray, end_left: np.ndarray):
        self.left = LineString3d(getId(), [Point3d(getId(), *end_left), Point3d(getId(), *start_left)])
        self.left.attributes["type"] = "road_border"

        self.right = LineString3d(getId(), [Point3d(getId(), *start_right), Point3d(getId(), *end_right)])
        self.right.attributes["type"] = "road_border"

        center_start = (start_right + start_left) / 2
        center_end = (end_right + end_left) / 2
        self.center = LineString3d(getId(), [Point3d(getId(), *center_start), Point3d(getId(), *center_end)])
        self.center.attributes["subtype"] = "dashed"
        self.center.attributes["type"] = "line_thin"

        self.center_inversed = LineString3d(getId(), self.center.invert())
        self.center_inversed.attributes["subtype"] = "dashed"
        self.center_inversed.attributes["type"] = "line_thin"

        self.lane_right = Lanelet(getId(), self.center, self.right)
        self.lane_left = Lanelet(getId(), self.center_inversed, self.left)

    def add(self, lanelet_map: LaneletMap):
        lanelet_map.add(self.lane_right)
        lanelet_map.add(self.lane_left)

class FourWayIntersection:
    def __init__(self, top_left: np.ndarray, top_right: np.ndarray, bottom_left: np.ndarray, bottom_right: np.ndarray, lane_width = 0.3, precision = 6):
        # turn right
        p = [Point3d(getId(), top_left[0] + lane_width * np.cos(phi), top_left[1] + lane_width * np.sin(phi), 0) for phi in np.linspace(2 * np.pi, 1.5 * np.pi, precision)]
        self.top_left_right = Lanelet(getId(), LineString3d(getId(), p), LineString3d(getId(), [Point3d(getId(), *top_left)]))
        self.top_left_right.attributes["intersection"] = "yes"

        p = [Point3d(getId(), top_right[0] + lane_width * np.cos(phi), top_right[1] + lane_width * np.sin(phi), 0) for phi in np.linspace(1.5 * np.pi, np.pi, precision)]
        self.top_right_right = Lanelet(getId(), LineString3d(getId(), p), LineString3d(getId(), [Point3d(getId(), *top_right)]))
        self.top_right_right.attributes["intersection"] = "yes"

        p = [Point3d(getId(), bottom_left[0] + lane_width * np.cos(phi), bottom_left[1] + lane_width * np.sin(phi), 0) for phi in np.linspace(0.5 * np.pi, 0, precision)]
        self.bottom_left_right = Lanelet(getId(), LineString3d(getId(), p), LineString3d(getId(), [Point3d(getId(), *bottom_left)]))
        self.bottom_left_right.attributes["intersection"] = "yes"

        p = [Point3d(getId(), bottom_right[0] + lane_width * np.cos(phi), bottom_right[1] + lane_width * np.sin(phi), 0) for phi in np.linspace(np.pi, 0.5 * np.pi, precision)]
        self.bottom_right_right = Lanelet(getId(), LineString3d(getId(), p), LineString3d(getId(), [Point3d(getId(), *bottom_right)]))
        self.bottom_right_right.attributes["intersection"] = "yes"

        # turn left
        p = [Point3d(getId(), top_right[0] + lane_width * 2 * np.cos(phi), top_right[1] + lane_width * 2 * np.sin(phi), 0) for phi in np.linspace(np.pi, 1.5 * np.pi, precision)]
        self.top_left_left = Lanelet(getId(), LineString3d(getId(), self.top_right_right.leftBound.invert()), LineString3d(getId(), p))
        self.top_left_left.attributes["intersection"] = "yes"

        p = [Point3d(getId(), top_left[0] + lane_width * 2 * np.cos(phi), top_left[1] + lane_width * 2 * np.sin(phi), 0) for phi in np.linspace(1.5 * np.pi, 2 * np.pi, precision)]
        self.top_right_left = Lanelet(getId(), LineString3d(getId(), self.top_left_right.leftBound.invert()), LineString3d(getId(), p))
        self.top_right_left.attributes["intersection"] = "yes"

        p = [Point3d(getId(), bottom_right[0] + lane_width * 2 * np.cos(phi), bottom_right[1] + lane_width * 2 * np.sin(phi), 0) for phi in np.linspace(0.5 * np.pi, np.pi, precision)]
        self.bottom_left_left = Lanelet(getId(), LineString3d(getId(), self.bottom_right_right.leftBound.invert()), LineString3d(getId(), p))
        self.bottom_left_left.attributes["intersection"] = "yes"

        p = [Point3d(getId(), bottom_left[0] + lane_width * 2 * np.cos(phi), bottom_left[1] + lane_width * 2 * np.sin(phi), 0) for phi in np.linspace(0, 0.5 * np.pi, precision)]
        self.bottom_right_left = Lanelet(getId(), LineString3d(getId(), self.bottom_left_right.leftBound.invert()), LineString3d(getId(), p))
        self.bottom_right_left.attributes["intersection"] = "yes"

        # straight
        top_center = (top_left + top_right) / 2
        bottom_center = (bottom_left + bottom_right) / 2
        left_center = (top_left + bottom_left) / 2
        right_center = (bottom_right + top_right) / 2

        self.top_left_straight = Lanelet(getId(),
                                         LineString3d(getId(), [Point3d(getId(), *top_center), Point3d(getId(), *bottom_center)]),
                                         LineString3d(getId(), [Point3d(getId(), *top_left), Point3d(getId(), *bottom_left)]))
        self.top_left_straight.attributes["intersection"] = "yes"

        self.top_right_straight = Lanelet(getId(),
                                         LineString3d(getId(), self.top_left_straight.leftBound.invert()),
                                         LineString3d(getId(), [Point3d(getId(), *bottom_right), Point3d(getId(), *top_right)]))
        self.top_right_straight.attributes["intersection"] = "yes"

        self.bottom_left_straight = Lanelet(getId(),
                                         LineString3d(getId(), [Point3d(getId(), *left_center), Point3d(getId(), *right_center)]),
                                         LineString3d(getId(), [Point3d(getId(), *bottom_left), Point3d(getId(), *bottom_right)]))
        self.bottom_left_straight.attributes["intersection"] = "yes"

        self.bottom_right_straight = Lanelet(getId(),
                                          LineString3d(getId(), self.bottom_left_straight.leftBound.invert()),
                                          LineString3d(getId(), [Point3d(getId(), *top_right), Point3d(getId(), *top_left)]))
        self.bottom_right_straight.attributes["intersection"] = "yes"


    def add(self, lanelet_map: LaneletMap):
        lanelet_map.add(self.top_left_right)
        lanelet_map.add(self.top_right_right)
        lanelet_map.add(self.bottom_left_right)
        lanelet_map.add(self.bottom_right_right)

        lanelet_map.add(self.top_left_left)
        lanelet_map.add(self.top_right_left)
        lanelet_map.add(self.bottom_left_left)
        lanelet_map.add(self.bottom_right_left)

        lanelet_map.add(self.top_left_straight)
        lanelet_map.add(self.top_right_straight)
        lanelet_map.add(self.bottom_left_straight)
        lanelet_map.add(self.bottom_right_straight)


def calculate_and_add_centerlines(lanelet_map: LaneletMap):
    for l in lanelet_map.laneletLayer:
        centerline = LineString3d(getId(), [Point3d(getId(), p.x, p.y, 0) for p in l.centerline])
        centerline.attributes["type"] = "virtual"
        lanelet_map.add(centerline)
        l.centerline = centerline

def merge_equal_points(lanelet_map: LaneletMap):
    grouped_by_position = defaultdict(list)
    for p in lanelet_map.pointLayer:
        grouped_by_position[(round(p.x, 4), round(p.y, 4))].append(p)

    for k, v in grouped_by_position.items():
        if len(v) > 1:
            main = v[0].id

            for replace in v[1:]:
                replace.id = main

def add_vehicle_attributes(lanelet_map: LaneletMap):
    for l in lanelet_map.laneletLayer:
        l.attributes["vehicle"] = "yes"
        l.attributes["region"] = "de"
        l.attributes["one_way"] = "yes"
        l.attributes["location"] = "urban"

def main():
    lanelet_map = LaneletMap()
    # inner part
    TwoLaneStraight(np.array([-5, 0, 0], dtype=np.float64), np.array([-5, 0.6, 0], dtype=np.float64), np.array([5, 0, 0], dtype=np.float64), np.array([5, 0.6, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([5.6, 0.6, 0], dtype=np.float64), np.array([5, 0.6, 0], dtype=np.float64), np.array([5.6, 5.6, 0], dtype=np.float64), np.array([5, 5.6, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([5.6, 0, 0], dtype=np.float64), np.array([5.6, 0.6, 0], dtype=np.float64), np.array([15.6, 0, 0], dtype=np.float64), np.array([15.6, 0.6, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([5.6, -5, 0], dtype=np.float64), np.array([5, -5, 0], dtype=np.float64), np.array([5.6, 0, 0], dtype=np.float64), np.array([5, 0, 0], dtype=np.float64)).add(lanelet_map)

    FourWayIntersection(np.array([5, 0.6, 0], dtype=np.float64), np.array([5.6, 0.6, 0], dtype=np.float64), np.array([5, 0, 0], dtype=np.float64), np.array([5.6, 0, 0], dtype=np.float64)).add(lanelet_map)

    # outer part
    TwoLaneStraight(np.array([-5, -5.6, 0], dtype=np.float64), np.array([-5, -5, 0], dtype=np.float64), np.array([5, -5.6, 0], dtype=np.float64), np.array([5, -5, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([-5, -5, 0], dtype=np.float64), np.array([-5.6, -5, 0], dtype=np.float64), np.array([-5, 0, 0], dtype=np.float64), np.array([-5.6, 0, 0], dtype=np.float64)).add(lanelet_map)

    TwoLaneStraight(np.array([-5, 5.6, 0], dtype=np.float64), np.array([-5, 6.2, 0], dtype=np.float64), np.array([5, 5.6, 0], dtype=np.float64), np.array([5, 6.2, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([-5, 0.6, 0], dtype=np.float64), np.array([-5.6, 0.6, 0], dtype=np.float64), np.array([-5, 5.6, 0], dtype=np.float64), np.array([-5.6, 5.6, 0], dtype=np.float64)).add(lanelet_map)

    TwoLaneStraight(np.array([5.6, 5.6, 0], dtype=np.float64), np.array([5.6, 6.2, 0], dtype=np.float64), np.array([15.6, 5.6, 0], dtype=np.float64), np.array([15.6, 6.2, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([15.6, 5.6, 0], dtype=np.float64), np.array([16.2, 5.6, 0], dtype=np.float64), np.array([15.6, 0.6, 0], dtype=np.float64), np.array([16.2, 0.6, 0], dtype=np.float64)).add(lanelet_map)

    TwoLaneStraight(np.array([15.6, 0, 0], dtype=np.float64), np.array([16.2, 0, 0], dtype=np.float64), np.array([15.6, -5, 0], dtype=np.float64), np.array([16.2, -5, 0], dtype=np.float64)).add(lanelet_map)
    TwoLaneStraight(np.array([15.6, -5, 0], dtype=np.float64), np.array([15.6, -5.6, 0], dtype=np.float64), np.array([5.6, -5, 0], dtype=np.float64), np.array([5.6, -5.6, 0], dtype=np.float64)).add(lanelet_map)

    FourWayIntersection(np.array([-5.6, -5, 0], dtype=np.float64), np.array([-5, -5, 0], dtype=np.float64), np.array([-5.6, -5.6, 0], dtype=np.float64), np.array([-5, -5.6, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([-5.6, 0.6, 0], dtype=np.float64), np.array([-5, 0.6, 0], dtype=np.float64), np.array([-5.6, 0, 0], dtype=np.float64), np.array([-5, 0, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([-5.6, 6.2, 0], dtype=np.float64), np.array([-5, 6.2, 0], dtype=np.float64), np.array([-5.6, 5.6, 0], dtype=np.float64), np.array([-5, 5.6, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([5, 6.2, 0], dtype=np.float64), np.array([5.6, 6.2, 0], dtype=np.float64), np.array([5, 5.6, 0], dtype=np.float64), np.array([5.6, 5.6, 0], dtype=np.float64)).add(lanelet_map)

    FourWayIntersection(np.array([15.6, 6.2, 0], dtype=np.float64), np.array([16.2, 6.2, 0], dtype=np.float64), np.array([15.6, 5.6, 0], dtype=np.float64), np.array([16.2, 5.6, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([15.6, 0.6, 0], dtype=np.float64), np.array([16.2, 0.6, 0], dtype=np.float64), np.array([15.6, 0, 0], dtype=np.float64), np.array([16.2, 0, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([15.6, -5, 0], dtype=np.float64), np.array([16.2, -5, 0], dtype=np.float64), np.array([15.6, -5.6, 0], dtype=np.float64), np.array([16.2, -5.6, 0], dtype=np.float64)).add(lanelet_map)
    FourWayIntersection(np.array([5, -5, 0], dtype=np.float64), np.array([5.6, -5, 0], dtype=np.float64), np.array([5, -5.6, 0], dtype=np.float64), np.array([5.6, -5.6, 0], dtype=np.float64)).add(lanelet_map)

    calculate_and_add_centerlines(lanelet_map)
    merge_equal_points(lanelet_map)
    add_vehicle_attributes(lanelet_map)

    path = 'mapfile.osm'
    projector = UtmProjector(Origin(52, 13, 0))
    write(path, lanelet_map, projector)


if __name__ == "__main__":
    main()
