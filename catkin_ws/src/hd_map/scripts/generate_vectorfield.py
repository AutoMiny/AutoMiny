import lanelet2
import numpy as np
import matplotlib.pyplot as plt
import h5py

import lanelet2.core
import lanelet2.routing
import lanelet2.traffic_rules
from lanelet2.io import Origin, load
from lanelet2.core import Lanelet, BasicPoint3d, CompoundLineString2d
from lanelet2.geometry import boundingBox2d, distance, length, to2D, toArcCoordinates, fromArcCoordinates, project
from lanelet2.projection import UtmProjector

class VectorFieldGenerator:
    def __init__(self, map):
        self.map = map
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = lanelet2.routing.RoutingGraph(map, traffic_rules)

    def bbox(self, lanelet):
        previous = self.graph.previous(lanelet)

        bboxes = [boundingBox2d(p) for p in previous]
        bboxes.append(boundingBox2d(lanelet))

        spanning_box = np.array([[bboxes[0].min.x, bboxes[0].min.y], [bboxes[0].max.x, bboxes[0].max.y]])
        for b in bboxes:
            spanning_box[0][0] = min(spanning_box[0][0], b.min.x)
            spanning_box[0][1] = min(spanning_box[0][1], b.min.y)
            spanning_box[1][0] = max(spanning_box[1][0], b.max.x)
            spanning_box[1][1] = max(spanning_box[1][1], b.max.y)

        return spanning_box

    def get_xy(self, bbox, precision=0.01):
        return np.meshgrid(np.arange(bbox[0][0], bbox[1][0] + precision, precision), np.arange(bbox[0][1], bbox[1][1] + precision, precision))

    def generate(self, lanelet_id, lookahead=0.3, precision=0.01):
        lane: Lanelet
        lane = self.map.laneletLayer[lanelet_id]
        previous = self.graph.previous(lane)

        spanning_box = self.bbox(lane)
        x, y = self.get_xy(spanning_box, precision)

        coords = np.vstack([x.ravel(), y.ravel()]).T
        closest_x = np.zeros_like(x).flatten()
        closest_y = np.zeros_like(y).flatten()

        i = 0
        for c in coords:
            possible_closest = [(p.centerline, project(p.centerline, BasicPoint3d(*c, 0))) for p in previous]
            possible_closest.append((lane.centerline, project(lane.centerline, BasicPoint3d(*c, 0))))

            closest_lane, closest_point = min(possible_closest, key=lambda pc: distance(BasicPoint3d(*c, 0), pc[1]))

            # create compound lane string if necessary, that is if closest point is on a previous lane
            if closest_lane != lane.centerline:
                compound = CompoundLineString2d([to2D(closest_lane), to2D(lane.centerline)])
            else:
                compound = to2D(lane.centerline)
            arc_coordinates = toArcCoordinates(compound, to2D(closest_point))
            # calculate lookahead and make sure that the lookahead point does not overflow the lane's length
            arc_coordinates.length = min(arc_coordinates.length + lookahead, length(compound))
            arc_coordinates.distance = 0
            lookahead_point = fromArcCoordinates(compound, arc_coordinates)

            closest_x[i] = lookahead_point.x
            closest_y[i] = lookahead_point.y
            i += 1

        u = closest_x.reshape(x.shape) - x
        v = closest_y.reshape(y.shape) - y

        dist = np.array([closest_x.reshape(x.shape), closest_y.reshape(y.shape)]).T - np.array([lane.centerline[-1].x, lane.centerline[-1].y])
        dist = np.sqrt(dist ** 2).sum(-1)[..., np.newaxis]

        # normalize vectors
        vec = np.array([u, v]).T
        vec /= np.sqrt(vec ** 2).sum(-1)[..., np.newaxis]

        dist[np.where(dist[:, :, 0] > 1)] = 1
        dist[np.where(dist[:, :, 0] < 1)] = dist[np.where(dist[:, :, 0] < 1)] ** 2
        return vec * dist

    def save(self, file, forces, lane_id, precision=0.01):
        group = file.create_group(str(lane_id))
        dset = group.create_dataset("forces", forces.shape, data=forces)
        dset.attrs["bbox"] = self.bbox(self.map.laneletLayer[lane_id])
        dset.attrs["precision"] = precision

    def visualize(self, forces, lane_id, precision=0.01):
        lane = self.map.laneletLayer[lane_id]
        x, y = self.get_xy(self.bbox(lane), precision)
        previous = self.graph.previous(lane)
        plt.quiver(x, y, forces.T[0], forces.T[1], angles='xy', scale_units='xy', scale=100)
        plt.plot([p.x for p in lane.centerline], [p.y for p in lane.centerline])
        for prev in previous:
            plt.plot([p.x for p in prev.centerline], [p.y for p in prev.centerline])
        plt.xlim(5.1, 5.7)
        plt.show()


if __name__ == "__main__":
    projector = UtmProjector(Origin(52, 13, 0))
    map = load("mapfile.osm", projector)
    generator = VectorFieldGenerator(map)

    file = h5py.File("vectorfield.hdf5", "w")
    for lanelet in map.laneletLayer:
        if "intersection" in lanelet.attributes and lanelet.attributes["intersection"] == "yes":
            continue

        print("Generating forces for lane {}".format(lanelet.id))
        forces = generator.generate(lanelet.id, lookahead=0.3, precision=0.01)
        #generator.visualize(forces, 1034)
        generator.save(file, forces, lanelet.id, precision=0.01)

    file.close()