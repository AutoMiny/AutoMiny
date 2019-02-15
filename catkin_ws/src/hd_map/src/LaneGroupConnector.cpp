#include <hd_map/LaneGroupConnector.h>

namespace hd_map {

    LaneGroupConnector::LaneGroupConnector(unsigned long id, const Polyline2DPtr &boundaryGeometry,
                                           const std::vector<LaneGroupPtr> &connectedLaneGroups) : id(id), boundaryGeometry(boundaryGeometry), connectedLaneGroups(connectedLaneGroups) {

    }

    LaneGroupConnector::LaneGroupConnector(unsigned long id, const Polyline2DPtr &boundaryGeometry) : id(id), boundaryGeometry(boundaryGeometry) {

    }


    const Polyline2DPtr hd_map::LaneGroupConnector::getBoundaryGeometry() {
        return this->boundaryGeometry;
    }

    unsigned long LaneGroupConnector::getId() {
        return this->id;
    }

    const std::vector<LaneGroupPtr>& LaneGroupConnector::getConnectedLaneGroups() {
        return this->connectedLaneGroups;
    }


    void LaneGroupConnector::setConnectedLaneGroups(const std::vector<LaneGroupPtr> &connectedLaneGroups) {
        this->connectedLaneGroups = connectedLaneGroups;
    }
}