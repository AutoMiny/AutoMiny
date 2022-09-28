#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"

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
        impl() = default;

        void loadMap();
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
        Graph mapGraph;
        /// map from lane id to graph vertex
        std::unordered_map<LanePtr, boost::graph_traits<Graph>::vertex_descriptor> verticesById;

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

    void HDMap::loadMap() {
        pimpl->loadMap();
    }

//endregion

    void HDMap::impl::loadMap() {

        {
            // Lanegroup 1
            auto laneGroup = std::make_shared<LaneGroup>(0);
            {
                std::vector <tf2::Vector3> referencePoints, leftPoints, rightPoints;

                referencePoints.emplace_back(1.95, 0.465, 0.0);
                referencePoints.emplace_back(4.04, 0.465, 0.0);
                leftPoints.emplace_back(1.95, 0.62, 0.0);
                leftPoints.emplace_back(4.04, 0.62, 0.0);
                rightPoints.emplace_back(1.95, 0.31, 0.0);
                rightPoints.emplace_back(4.04, 0.31, 0.0);

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(0, laneGroup, reference, left, right);
                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            {
                std::vector <tf2::Vector3> referencePoints, leftPoints, rightPoints;

                referencePoints.emplace_back(1.95, 0.785, 0.0);
                referencePoints.emplace_back(4.04, 0.785, 0.0);
                leftPoints.emplace_back(1.95, 0.94, 0.0);
                leftPoints.emplace_back(4.04, 0.94, 0.0);
                rightPoints.emplace_back(1.95, 0.63, 0.0);
                rightPoints.emplace_back(4.04, 0.63, 0.0);

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(1, laneGroup, reference, left, right);
                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }

            laneGroupsById.emplace(laneGroup->getId(), laneGroup);
        }

        {
            // Lanegroup 1
            auto laneGroup = std::make_shared<LaneGroup>(1);
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;

                auto xo = 4.04;
                auto yo = 2.15;
                auto refR = 1.685;
                auto leftR = 1.84;
                auto rightR = 1.53;
                for (double i = -M_PI / 2.0; i <= (M_PI / 2.0) + 0.009; i += 1 / (refR * 50)) {
                    auto xp = xo + refR * std::cos(i);
                    auto yp = yo + refR * std::sin(i);
                    referencePoints.emplace_back(xp, yp, 0);

                    xp = xo + leftR * std::cos(i);
                    yp = yo + leftR * std::sin(i);
                    leftPoints.emplace_back(xp, yp, 0);

                    xp = xo + rightR * std::cos(i);
                    yp = yo + rightR * std::sin(i);
                    rightPoints.emplace_back(xp, yp, 0);
                }

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(0, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;

                auto xo = 4.04;
                auto yo = 2.15;
                auto refR = 1.365;
                auto leftR = 1.21;
                auto rightR = 1.52;
                for (double i = -M_PI / 2.0; i <= (M_PI / 2.0) + 0.009; i += 1 / (refR * 50)) {
                    auto xp = xo + refR * std::cos(i);
                    auto yp = yo + refR * std::sin(i);
                    referencePoints.emplace_back(xp, yp, 0);

                    xp = xo + leftR * std::cos(i);
                    yp = yo + leftR * std::sin(i);
                    leftPoints.emplace_back(xp, yp, 0);

                    xp = xo + rightR * std::cos(i);
                    yp = yo + rightR * std::sin(i);
                    rightPoints.emplace_back(xp, yp, 0);
                }

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(1, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            laneGroupsById.emplace(laneGroup->getId(), laneGroup);
        }

        {
            // Lanegroup 2
            auto laneGroup = std::make_shared<LaneGroup>(2);
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;
                referencePoints.emplace_back(4.04, 3.835, 0.0);
                referencePoints.emplace_back(1.95, 3.835, 0.0);
                leftPoints.emplace_back(4.04, 3.99, 0.0);
                leftPoints.emplace_back(1.95, 3.99, 0.0);
                rightPoints.emplace_back(4.04, 3.68, 0.0);
                rightPoints.emplace_back(1.95, 3.68, 0.0);

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(0, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;
                referencePoints.emplace_back(4.04, 3.515, 0.0);
                referencePoints.emplace_back(1.95, 3.515, 0.0);
                leftPoints.emplace_back(4.04, 3.36, 0.0);
                leftPoints.emplace_back(1.95, 3.36, 0.0);
                rightPoints.emplace_back(4.04, 3.67, 0.0);
                rightPoints.emplace_back(1.95, 3.67, 0.0);

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(1, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            laneGroupsById.emplace(laneGroup->getId(), laneGroup);
        }

        {
            // Lanegroup 4
            auto laneGroup = std::make_shared<LaneGroup>(3);
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;

                auto xo = 1.95;
                auto yo = 2.15;
                auto refR = 1.685;
                auto leftR = 1.84;
                auto rightR = 1.53;
                for (double i = M_PI / 2.0; i <= 3 * (M_PI / 2.0) + 0.009; i += 1 / (refR * 50)) {
                    auto xp = xo + refR * std::cos(i);
                    auto yp = yo + refR * std::sin(i);
                    referencePoints.emplace_back(xp, yp, 0);

                    xp = xo + leftR * std::cos(i);
                    yp = yo + leftR * std::sin(i);
                    leftPoints.emplace_back(xp, yp, 0);

                    xp = xo + rightR * std::cos(i);
                    yp = yo + rightR * std::sin(i);
                    rightPoints.emplace_back(xp, yp, 0);
                }

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(0, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            {
                std::vector<tf2::Vector3> referencePoints, leftPoints, rightPoints;

                auto xo = 1.95;
                auto yo = 2.15;
                auto refR = 1.365;
                auto leftR = 1.21;
                auto rightR = 1.52;
                for (double i = M_PI / 2.0; i <= 3 * (M_PI / 2.0) + 0.009; i += 1 / (refR * 50)) {
                    auto xp = xo + refR * std::cos(i);
                    auto yp = yo + refR * std::sin(i);
                    referencePoints.emplace_back(xp, yp, 0);

                    xp = xo + leftR * std::cos(i);
                    yp = yo + leftR * std::sin(i);
                    leftPoints.emplace_back(xp, yp, 0);

                    xp = xo + rightR * std::cos(i);
                    yp = yo + rightR * std::sin(i);
                    rightPoints.emplace_back(xp, yp, 0);
                }

                auto reference = std::make_shared<Polyline3D>(referencePoints);
                auto left = std::make_shared<Polyline3D>(leftPoints);
                auto right = std::make_shared<Polyline3D>(rightPoints);
                auto lane = std::make_shared<Lane>(1, laneGroup, reference, left, right);

                laneGroup->addLane(lane);
                lane->setLaneType(LaneType::REGULAR);
                lane->setRelativeDirection(RelativeDirection::FORWARD);
            }
            laneGroupsById.emplace(laneGroup->getId(), laneGroup);
        }

        std::vector<LaneGroupPtr> g0, g1, g2, g3;
        g0.emplace_back(laneGroupsById[0]);
        g1.emplace_back(laneGroupsById[1]);
        g2.emplace_back(laneGroupsById[2]);
        g3.emplace_back(laneGroupsById[3]);

        laneGroupsById[0]->setOutgoingLaneGroups(g1);
        laneGroupsById[1]->setOutgoingLaneGroups(g2);
        laneGroupsById[2]->setOutgoingLaneGroups(g3);
        laneGroupsById[3]->setOutgoingLaneGroups(g0);

        laneGroupsById[0]->setIncomingLaneGroups(g3);
        laneGroupsById[1]->setIncomingLaneGroups(g0);
        laneGroupsById[2]->setIncomingLaneGroups(g1);
        laneGroupsById[3]->setIncomingLaneGroups(g2);

        laneGroupsById[0]->getLane(0)->addOutgoingLane(laneGroupsById[1]->getLane(0));
        laneGroupsById[1]->getLane(0)->addOutgoingLane(laneGroupsById[2]->getLane(0));
        laneGroupsById[2]->getLane(0)->addOutgoingLane(laneGroupsById[3]->getLane(0));
        laneGroupsById[3]->getLane(0)->addOutgoingLane(laneGroupsById[0]->getLane(0));

        laneGroupsById[0]->getLane(1)->addOutgoingLane(laneGroupsById[1]->getLane(1));
        laneGroupsById[1]->getLane(1)->addOutgoingLane(laneGroupsById[2]->getLane(1));
        laneGroupsById[2]->getLane(1)->addOutgoingLane(laneGroupsById[3]->getLane(1));
        laneGroupsById[3]->getLane(1)->addOutgoingLane(laneGroupsById[0]->getLane(1));

        buildGraph();
    }

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
        auto currentDistance = DBL_MAX;
        LanePtr closestLane = nullptr;

        for (const auto &laneGroup : getLaneGroups()) {
            for (const auto &lane : laneGroup->getLanes()) {
                auto param = lane->getReferenceTrack()->findClosestParameter(point);
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
        auto currentDistance = DBL_MAX;
        LanePtr closestLane = nullptr;

        for (const auto &laneGroup : getLaneGroups()) {
            for (const auto &lane : laneGroup->getLanes()) {
                auto param = lane->getReferenceTrack()->findClosestParameter(point);
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
                auto param = lane->getReferenceTrack()->findClosestParameter(point);
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
                if (verticesById.find(lane) == verticesById.end()) {
                    boost::graph_traits<Graph>::vertex_descriptor v = boost::add_vertex(lane, mapGraph);
                    verticesById[lane] = v;
                }

                // add connections to next road section lanes
                for (const auto& followingLane : lane->getOutgoingLanes()) {
                    // add not already inserted vertices
                    if (verticesById.find(followingLane) == verticesById.end()) {
                        boost::graph_traits<Graph>::vertex_descriptor v_out = boost::add_vertex(followingLane, mapGraph);
                        verticesById[followingLane] = v_out;
                    }

                    // add connecting edge
                    EdgeWeightProperty weightProperty = followingLane->getReferenceTrack()->length();
                    boost::add_edge(verticesById[lane], verticesById[followingLane], weightProperty,
                                    mapGraph);
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
        boost::graph_traits<Graph>::vertex_descriptor startVertex = verticesById[startLane];
        boost::graph_traits<Graph>::vertex_descriptor goalVertex = verticesById[endLane];

        // backtracking from goal vertex to start
        std::vector<LanePtr> path;

        if (startLane == endLane) {
            path.push_back(mapGraph[startVertex]);
            return path;
        }

        std::vector<boost::graph_traits<Graph>::vertex_descriptor> predecessors(boost::num_vertices(mapGraph));
        std::vector<double> distances(boost::num_vertices(mapGraph));

        boost::dijkstra_shortest_paths(mapGraph, startVertex,
                                       boost::predecessor_map(&predecessors[0]).distance_map(&distances[0]));

        // first entry of predecessors links to itself
        while (goalVertex != predecessors[goalVertex]) {
            if (mapGraph[goalVertex] != mapGraph[predecessors[goalVertex]]) {
                path.push_back(mapGraph[goalVertex]);
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
