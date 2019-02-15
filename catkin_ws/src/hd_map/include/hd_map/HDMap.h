#pragma once

#include <vector>
#include <unordered_map>
#include <tf2/utils.h>
#include "Fwd.h"

namespace hd_map {
    class HDMap {
    public:
        HDMap();
        ~HDMap();
        HDMap(HDMap&&);
        HDMap& operator=(HDMap&&);

        const std::vector<RoadPtr> getRoads();
        const std::vector<LaneGroupPtr> getLaneGroups();

        /**
         * Gets the closest lane given a position.
         * @param point The point to look for
         * @param closestPoint The closest point on the returned closest lane
         * @return Closest lane closest to point
         */
        const LanePtr closestLane(const tf2::Vector3& point, tf2::Vector3& closestPoint);

        /**
         * Gets the closest lane given a position.
         * @param point The point to look for
         * @return Closest lane closest to point
         */
        const LanePtr closestLane(const tf2::Vector3& point);

        /**
         * Gets the closest lane given a position. The closest lane must either be of type none or the specified lane type
         * @param point The point to look for
         * @param type Type filter
         * @return Closest lane with type none or specified type closest to point
         */
        const LanePtr closestLane(const tf2::Vector3& point, const LaneType& type);

        /**
         * Gets the closest lanes given a position.
         * @param point The point to look for
         * @param num The maximum number of closest lanes to search
         * @return Closest lanes closest to point ordered by their distance (maximum num entries)
         */
        const std::vector<LanePtr> closestLanes(const tf2::Vector3& point, int num);

        /**
         * Finds a lane route between two points. There will be no lane changes.
         * @param start Start position to use for closest lane
         * @param end End position to use for closest lane
         * @return Sequence of lanes from start to end to reach end
         */
        const std::vector<LanePtr> findRoute(const tf2::Vector3& start, const tf2::Vector3& end);

        /**
         * Finds a lane route between two lanes. There will be no lane changes.
         * @param start Start lane
         * @param end End lane
         * @return Sequence of lanes from start to end to reach end
         */
        const std::vector<LanePtr> findRoute(const LanePtr& start, const LanePtr& end);

        /**
         * Finds a road by id
         * @param id HERE road id
         * @return RoadPtr if the road was found, else nullptr
         */
        const RoadPtr getRoad(unsigned int id);

        /**
         * Finds a lane group by id
         * @param id HERE lane group id
         * @return LaneGroupPtr if the lane group was found, else nullptr
         */
        const LaneGroupPtr getLaneGroup(unsigned int id);
    private:
        class impl;
        std::unique_ptr<impl> pimpl;
    };
}

