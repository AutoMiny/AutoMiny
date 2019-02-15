#include <vector>
#include <hd_map/LaneGroup.h>
#include <hd_map/Lane.h>
#include <hd_map/LaneGroupConnector.h>
#include <hd_map/Polyline3D.h>

namespace hd_map {
    const Polyline2DPtr& LaneGroup::getLeftBoundary() {
        return this->leftBoundary;
    }

    const Polyline2DPtr& LaneGroup::getRightBoundary() {
        return this->rightBoundary;
    }

    const std::vector<LanePtr>& LaneGroup::getLanes() {
        return this->lanes;
    }

    const LanePtr LaneGroup::getLane(unsigned int id) {
        for (const auto& lane : this->lanes) {
            if (lane->getId() == id) {
                return lane;
            }
        }

        return nullptr;
    }

    LaneGroup::LaneGroup(unsigned long id) : id(id) {
    }

    LaneGroup::LaneGroup(unsigned long id, Polyline2DPtr& leftBoundary, Polyline2DPtr& rightBoundary, std::vector<std::tuple<unsigned int, unsigned int>>& laneMapping) : id(id), rightBoundary(rightBoundary), leftBoundary(leftBoundary), laneMapping(laneMapping) {
    }

    void LaneGroup::addLane(const LanePtr& lane) {
        this->lanes.push_back(lane);
    }

    unsigned long LaneGroup::getId() {
        return this->id;
    }

    void LaneGroup::setLeftBoundary(const Polyline2DPtr &leftBoundary) {
        this->leftBoundary = leftBoundary;
    }

    void LaneGroup::setRightBoundary(const Polyline2DPtr &rightBoundary) {
        this->rightBoundary = rightBoundary;
    }

    const std::vector<LaneGroupPtr> LaneGroup::getOutgoingLaneGroups() {
        return this->outgoingLaneGroups;
    }

    const std::vector<LaneGroupPtr> LaneGroup::getIncomingLaneGroups() {
        return this->incomingLaneGroups;
    }

    void LaneGroup::setOutgoingLaneGroups(const std::vector<LaneGroupPtr> &outgoingLaneGroups) {
        this->outgoingLaneGroups = outgoingLaneGroups;
    }

    void LaneGroup::setIncomingLaneGroups(const std::vector<LaneGroupPtr> &incomingLaneGroups) {
        this->incomingLaneGroups = incomingLaneGroups;
    }

    const LaneGroupConnectorPtr &LaneGroup::getStartConnector() {
        return this->startConnector;
    }

    const LaneGroupConnectorPtr &LaneGroup::getEndConnector() {
        return this->endConnector;
    }

    void LaneGroup::setStartConnector(const LaneGroupConnectorPtr& startConnector) {
        this->startConnector = startConnector;
    }

    void LaneGroup::setEndConnector(const LaneGroupConnectorPtr& endConnector) {
        this->endConnector = endConnector;
    }

    std::vector<std::tuple<unsigned int, unsigned int>> &LaneGroup::getLaneMapping() {
        return this->laneMapping;
    }

    bool LaneGroup::isJunction() {
        for (const auto& lane : lanes) {
            if (lane->getReferenceTrack()->length() < 25) {
                return true;
            }
        }

        return false;
    }
}