#pragma once

#include <hd_map/Fwd.h>
#include <hd_map/LaneGroup.h>

namespace hd_map {
    class LaneGroupConnector {
    public:
        explicit LaneGroupConnector(unsigned long id, const Polyline2DPtr& boundaryGeometry, const std::vector<LaneGroupPtr>& connectingLaneGroups);
        explicit LaneGroupConnector(unsigned long id, const Polyline2DPtr& boundaryGeometry);
        const std::vector<LaneGroupPtr>& getConnectedLaneGroups();
        void setConnectedLaneGroups(const std::vector<LaneGroupPtr> &connectedLaneGroups);
        unsigned long getId();
        const Polyline2DPtr getBoundaryGeometry();
    private:
        unsigned long id;
        Polyline2DPtr boundaryGeometry;
        std::vector<LaneGroupPtr> connectedLaneGroups;
    };
}