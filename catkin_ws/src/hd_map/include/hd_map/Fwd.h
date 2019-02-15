#pragma once

#include <memory>

namespace hd_map {
    class Road;
    class Lane;
    class LaneGroup;
    class LaneGroupConnector;
    class Polyline2D;
    class Polyline3D;
    enum class RelativeDirection;
    enum class LaneType;
    enum class GeodeticType;

    typedef std::shared_ptr<Road> RoadPtr;
    typedef std::shared_ptr<Lane> LanePtr;
    typedef std::shared_ptr<LaneGroup> LaneGroupPtr;
    typedef std::shared_ptr<LaneGroupConnector> LaneGroupConnectorPtr;
    typedef std::shared_ptr<Polyline2D> Polyline2DPtr;
    typedef std::shared_ptr<Polyline3D> Polyline3DPtr;
}