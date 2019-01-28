#pragma once

#include <memory>

namespace road_marking_localization {

/** Forward declaration of the Dummy class with type aliases.
 **
 ** If the Dummy class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class RoadMarkingLocalization;

    using RoadMarkingLocalizationUniquePtr = std::unique_ptr<RoadMarkingLocalization>;
    using RoadMarkingLocalizationUniqueConstPtr = std::unique_ptr<const RoadMarkingLocalization>;
    using RoadMarkingLocalizationPtr = std::shared_ptr<RoadMarkingLocalization>;
    using RoadMarkingLocalizationConstPtr = std::shared_ptr<const RoadMarkingLocalization>;
}
