#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <hd_map/HDMap.h>
#include <hd_map/Polyline2D.h>
#include <hd_map/Polyline3D.h>
#include <hd_map/LaneGroup.h>
#include <hd_map/LaneGroupConnector.h>
#include "hd_map/Road.h"
#include "hd_map/Lane.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

namespace hd_map {

    class HDMap::impl {
    public:
        impl() {};

        const std::vector<RoadPtr> getRoads();
        const std::vector<LaneGroupPtr> getLaneGroups();
        const LanePtr closestLane(const tf2::Vector3& point, tf2::Vector3& closestPoint);
        const LanePtr closestLane(const tf2::Vector3& point);
        const LanePtr closestLane(const tf2::Vector3& point, const LaneType& type);
        const std::vector<LanePtr> closestLanes(const tf2::Vector3& point, int num);
        const std::vector<LanePtr> findRoute(const tf2::Vector3& start, const tf2::Vector3& end);
        const std::vector<LanePtr> findRoute(const LanePtr& startLane, const LanePtr& endLane);
        const LaneGroupPtr getLaneGroup(unsigned long id);
        const RoadPtr getRoad(unsigned long id);
    private:
        typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, LanePtr, EdgeWeightProperty> Graph;
        /// internal boost graph representation of the map
        Graph graph_;
        /// map from lane id to graph vertex
        std::unordered_map<LanePtr, boost::graph_traits<Graph>::vertex_descriptor> vertices_by_id_;

        void buildGraph();

        std::unordered_map<unsigned long, RoadPtr> roadsById;
        std::unordered_map<unsigned long, LaneGroupPtr> laneGroupsById;
        std::unordered_map<unsigned long, LaneGroupConnectorPtr> laneGroupConnectorsById;
    };

//region pimpl
    HDMap::HDMap() : pimpl(std::make_unique<impl>()) {}
    HDMap::~HDMap() = default;
    HDMap::HDMap(HDMap&&) = default;
    HDMap& HDMap::operator=(HDMap&&) = default;

    const std::vector<RoadPtr> HDMap::getRoads() {
        return pimpl->getRoads();
    }

    const std::vector<LaneGroupPtr> HDMap::getLaneGroups() {
        return pimpl->getLaneGroups();
    }

    const LanePtr HDMap::closestLane(const tf2::Vector3& point, tf2::Vector3& closestPoint) {
        return pimpl->closestLane(point, closestPoint);
    }

    const LanePtr HDMap::closestLane(const tf2::Vector3& point) {
        return pimpl->closestLane(point);
    }

    const LanePtr HDMap::closestLane(const tf2::Vector3& point, const LaneType& type) {
        return pimpl->closestLane(point, type);
    }

    const std::vector<LanePtr> HDMap::closestLanes(const tf2::Vector3& point, int num) {
        return pimpl->closestLanes(point, num);
    }

    const std::vector<LanePtr> HDMap::findRoute(const tf2::Vector3& start, const tf2::Vector3& end) {
        return pimpl->findRoute(start, end);
    }

    const std::vector<LanePtr> HDMap::findRoute(const LanePtr& start, const LanePtr& end) {
        return pimpl->findRoute(start, end);
    }

    const RoadPtr HDMap::getRoad(unsigned int id) {
        return pimpl->getRoad(id);
    }

    const LaneGroupPtr HDMap::getLaneGroup(unsigned int id) {
        return pimpl->getLaneGroup(id);
    }

//endregion

    const std::vector<RoadPtr> HDMap::impl::getRoads() {
        std::vector<RoadPtr> roads;
        roads.reserve(roadsById.size());

        for (const auto &kv : roadsById) {
            roads.push_back(kv.second);
        }

        return roads;
    }

    const std::vector<LaneGroupPtr> HDMap::impl::getLaneGroups() {
        std::vector<LaneGroupPtr> laneGroups;
        laneGroups.reserve(laneGroupsById.size());

        for (const auto &kv : laneGroupsById) {
            laneGroups.push_back(kv.second);
        }

        return laneGroups;
    }

    const LanePtr HDMap::impl::closestLane(const tf2::Vector3& point) {
        auto tmp = tf2::Vector3(0, 0, 0);
        return closestLane(point, tmp);
    }

    const LanePtr HDMap::impl::closestLane(const tf2::Vector3& point, tf2::Vector3& closestPoint) {
        double currentDistance = DBL_MAX;
        LanePtr closestLane = nullptr;

        for (const auto &laneGroup : getLaneGroups()) {
            for (const auto &lane : laneGroup->getLanes()) {
                auto param = lane->getReferenceTrack()->findClosestParameter(point, 0.1);
                auto p2 = lane->getReferenceTrack()->interpolate(param);

                auto distance = point.distance(p2);

                if (distance < currentDistance) {
                    currentDistance = distance;
                    closestLane = lane;
                    closestPoint = p2;
                }
            }
        }
        return closestLane;
    }

    const LanePtr HDMap::impl::closestLane(const tf2::Vector3& point, const LaneType& type) {
        double currentDistance = DBL_MAX;
        LanePtr closestLane = nullptr;

        for (const auto &laneGroup : getLaneGroups()) {
            for (const auto &lane : laneGroup->getLanes()) {
                auto param = lane->getReferenceTrack()->findClosestParameter(point, 0.1);
                auto p2 = lane->getReferenceTrack()->interpolate(param);
                auto distance = point.distance(p2);
                auto laneType= lane->getLaneType();

                if (distance < currentDistance && (laneType == type || laneType == LaneType::UNKNOWN)) {
                    currentDistance = distance;
                    closestLane = lane;
                }
            }
        }
        return closestLane;
    }

    const std::vector<LanePtr> HDMap::impl::closestLanes(const tf2::Vector3& point, int num) {
        std::map<double, LanePtr> closestLanesByDistance;

        for (const auto &laneGroup : getLaneGroups()) {
            for (const auto &lane : laneGroup->getLanes()) {
                auto param = lane->getReferenceTrack()->findClosestParameter(point, 0.1);
                auto p2 = lane->getReferenceTrack()->interpolate(param);
                auto distance = point.distance(p2);

                closestLanesByDistance[distance] = lane;
            }
        }

        std::vector<LanePtr> ret;
        int i = 0;
        for (const auto& kv : closestLanesByDistance) {
            if (i++ > num) {
                break;
            }

            ret.emplace_back(kv.second);
        }

        return ret;
    }

    const LaneGroupPtr HDMap::impl::getLaneGroup(unsigned long id) {
        if (laneGroupsById.find(id) != laneGroupsById.end()) {
            return laneGroupsById[id];
        } else {
            return nullptr;
        }
    }

    const RoadPtr HDMap::impl::getRoad(unsigned long id) {
        if (roadsById.find(id) != roadsById.end()) {
            return roadsById[id];
        } else {
            return nullptr;
        }
    }

    void HDMap::impl::buildGraph() {
        for (const auto& laneGroup : getLaneGroups()) {
            for (const auto& lane : laneGroup->getLanes()) {
                if (vertices_by_id_.find(lane) == vertices_by_id_.end()) {
                    boost::graph_traits<Graph>::vertex_descriptor v = boost::add_vertex(lane, graph_);
                    vertices_by_id_[lane] = v;
                }

                // add connections to next road section lanes
                for (const auto& followingLane : lane->getOutgoingLanes()) {
                    // add not already inserted vertices
                    if (vertices_by_id_.find(followingLane) == vertices_by_id_.end()) {
                        boost::graph_traits<Graph>::vertex_descriptor v_out = boost::add_vertex(followingLane, graph_);
                        vertices_by_id_[followingLane] = v_out;
                    }

                    // add connecting edge
                    EdgeWeightProperty weightProperty = followingLane->getReferenceTrack()->length();
                    boost::add_edge(vertices_by_id_[lane], vertices_by_id_[followingLane], weightProperty,
                                    graph_);
                }

            }
        }
    }

    const std::vector<LanePtr> HDMap::impl::findRoute(const tf2::Vector3& start, const tf2::Vector3& end) {
        const auto& startLane = closestLane(start);
        const auto& endLane = closestLane(end);

        return findRoute(startLane, endLane);
    }

    const std::vector<LanePtr> HDMap::impl::findRoute(const LanePtr& startLane, const LanePtr& endLane) {
        boost::graph_traits<Graph>::vertex_descriptor startVertex = vertices_by_id_[startLane];
        boost::graph_traits<Graph>::vertex_descriptor goalVertex = vertices_by_id_[endLane];

        // backtracking from goal vertex to start
        std::vector<LanePtr> path;

        if (startLane == endLane) {
            path.push_back(graph_[startVertex]);
            return path;
        }

        std::vector<boost::graph_traits<Graph>::vertex_descriptor> predecessors(boost::num_vertices(graph_));
        std::vector<double> distances(boost::num_vertices(graph_));

        boost::dijkstra_shortest_paths(graph_, startVertex,
                                       boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));

        // first entry of predecessors links to itself
        while (goalVertex != predecessors[goalVertex]) {
            if (graph_[goalVertex] != graph_[predecessors[goalVertex]]) {
                path.push_back(graph_[goalVertex]);
            }
            goalVertex = predecessors[goalVertex];
        }
        // add start node because backtracking was stopped when we reached the start
        if (not path.empty()) {
            path.emplace_back(startLane);
        }

        // returned path is from goal to start -> reverse it
        std::reverse(path.begin(), path.end());

        return path;
    }
}
