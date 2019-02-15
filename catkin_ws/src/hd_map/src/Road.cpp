#include <utility>
#include <hd_map/Road.h>

namespace hd_map {
    Road::Road(unsigned long id, Polyline2DPtr referenceTrack) : id(id), referenceTrack(std::move(referenceTrack)) {

    }

    unsigned long Road::getId() {
        return this->id;
    }

    Polyline2DPtr Road::getReferenceTrack() {
        return this->referenceTrack;
    }
}
