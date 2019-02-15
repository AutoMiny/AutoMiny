#pragma once

#include <vector>

#include "Fwd.h"

namespace hd_map {
    class Road;
    typedef std::shared_ptr<Road> RoadPtr;

    class Road {
    public:
        explicit Road(unsigned long id, Polyline2DPtr referenceTrack);
        unsigned long getId();
        Polyline2DPtr getReferenceTrack();
    private:
        unsigned long id;
        std::vector<LanePtr> lanes;
        std::vector<RoadPtr> previousRoads;
        std::vector<RoadPtr> followingRoads;
        Polyline2DPtr referenceTrack;
    };

}
