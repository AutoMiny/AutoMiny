#pragma once

#include <vector>
#include <memory>
#include "Road.h"
#include "Polyline2D.h"

namespace hd_map {
    class Lane;
    typedef std::shared_ptr<Lane> LanePtr;

    enum class RelativeDirection {
        NONE,
        BOTH,
        FORWARD,
        BACKWARD
    };

    enum class LaneType {
        UNKNOWN,
        REGULAR,
        HIGH_OCCUPANCY_VEHICLE,
        DRIVABLE_SHOULDER,
        BICYCLE,
        PARKING,
        REVERSIBLE,
        EXPRESS,
        ACCELERATION,
        DECELERATION,
        AUXILIARY,
        SLOW,
        PASSING,
        REGULATED_ACCESS,
        TURN,
        CENTRE_TURN,
        TRUCK_PARKING,
        SHOULDER,
        VARIABLE_DRIVING,
        DRIVABLE_PARKING,
        OTHER
    };

    class Lane {
    public:
        explicit Lane(unsigned long id, const LaneGroupPtr& laneGroup, const Polyline3DPtr& referenceTrack, const Polyline3DPtr& leftBoundary, const Polyline3DPtr& rightBoundary);
        unsigned long getId();
        const Polyline3DPtr& getReferenceTrack();
        const Polyline3DPtr& getLeftBoundary();
        const Polyline3DPtr& getRightBoundary();
        const LaneGroupPtr& getLaneGroup();
        const std::vector<LanePtr>& getOutgoingLanes();
        const std::vector<LanePtr>& getIncomingLanes();
        const RelativeDirection& getRelativeDirection();
        void setRelativeDirection(const RelativeDirection& direction);
        const LaneType& getLaneType();
        void setLaneType(const LaneType& direction);
        void addOutgoingLane(const LanePtr& lane);
    private:
        unsigned long id;
        LaneType laneType;
        RelativeDirection direction;
        LaneGroupPtr laneGroup;
        std::vector<LanePtr> outgoingLanes;
        std::vector<LanePtr> incomingLanes;
        Polyline3DPtr leftBoundary;
        Polyline3DPtr rightBoundary;
        Polyline3DPtr referenceTrack;
    };

}
