#include <utility>

#include <utility>


#include <hd_map/Lane.h>
#include <hd_map/Polyline3D.h>

namespace hd_map {
    Lane::Lane(unsigned long id, const LaneGroupPtr& laneGroup, const Polyline3DPtr& referenceTrack, const Polyline3DPtr& leftBoundary, const Polyline3DPtr& rightBoundary)
    : id(id), laneGroup(laneGroup), referenceTrack(referenceTrack), leftBoundary(leftBoundary), rightBoundary(rightBoundary) {

    }

    unsigned long Lane::getId() {
        return this->id;
    }

    const Polyline3DPtr& Lane::getReferenceTrack() {
        return this->referenceTrack;
    }

    const Polyline3DPtr& Lane::getLeftBoundary() {
        return this->leftBoundary;
    }

    const Polyline3DPtr& Lane::getRightBoundary() {
        return this->rightBoundary;
    }

    const LaneGroupPtr& Lane::getLaneGroup() {
        return this->laneGroup;
    }

    const std::vector<LanePtr> &Lane::getOutgoingLanes() {
        return this->outgoingLanes;
    }

    const std::vector<LanePtr> &Lane::getIncomingLanes() {
        return this->incomingLanes;
    }

    void Lane::addOutgoingLane(const LanePtr &lane) {
        this->outgoingLanes.push_back(lane);
    }

    void Lane::setRelativeDirection(const RelativeDirection &direction) {
        this->direction = direction;

        if (direction == RelativeDirection::BACKWARD) {
            this->referenceTrack->reverse();
            this->leftBoundary->reverse();
            this->rightBoundary->reverse();
        }
    }

    const RelativeDirection &Lane::getRelativeDirection() {
        return direction;
    }

    void Lane::setLaneType(const LaneType &laneType) {
        this->laneType = laneType;
    }

    const LaneType& Lane::getLaneType() {
        return this->laneType;
    }

}
