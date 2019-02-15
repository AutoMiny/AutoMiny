#pragma once
#include "Fwd.h"
#include <vector>

namespace hd_map {
    class LaneGroup {
    public:
        explicit LaneGroup(unsigned long id, Polyline2DPtr& leftBoundary, Polyline2DPtr& rightBoundary, std::vector<std::tuple<unsigned int, unsigned int>>& laneMappings);
        explicit LaneGroup(unsigned long id);
        const Polyline2DPtr& getLeftBoundary();
        const Polyline2DPtr& getRightBoundary();
        void setLeftBoundary(const Polyline2DPtr& leftBoundary);
        void setRightBoundary(const Polyline2DPtr& rightBoundary);
        const std::vector<LaneGroupPtr> getOutgoingLaneGroups();
        const std::vector<LaneGroupPtr> getIncomingLaneGroups();
        void setOutgoingLaneGroups(const std::vector<LaneGroupPtr>& outgoingLaneGroups);
        void setIncomingLaneGroups(const std::vector<LaneGroupPtr>& incomingLaneGroups);
        unsigned long getId();
        const std::vector<LanePtr>& getLanes();
        void addLane(const LanePtr& lane);
        /**
         * Gets a lane by id
         * @param id HERE lane id
         * @return LanePtr if found, else nullptr
         */
        const LanePtr getLane(unsigned int id);
        const LaneGroupConnectorPtr& getStartConnector();
        const LaneGroupConnectorPtr& getEndConnector();
        void setStartConnector(const LaneGroupConnectorPtr& startConnector);
        void setEndConnector(const LaneGroupConnectorPtr& endConnector);
        std::vector<std::tuple<unsigned int, unsigned int>>& getLaneMapping();
        bool isJunction();
    private:
        unsigned long id;
        Polyline2DPtr leftBoundary;
        Polyline2DPtr rightBoundary;
        std::vector<LanePtr> lanes;
        std::vector<std::tuple<unsigned int, unsigned int>> laneMapping;
        std::vector<LaneGroupPtr> outgoingLaneGroups;
        std::vector<LaneGroupPtr> incomingLaneGroups;
        LaneGroupConnectorPtr startConnector;
        LaneGroupConnectorPtr endConnector;
    };
}