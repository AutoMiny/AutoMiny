#pragma once

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

namespace fub_speed_calibration {

class SpeedCalibrationData {

 public:
    SpeedCalibrationData() = default;
    SpeedCalibrationData(int command, double twist);

 private:
    friend class boost::serialization::access;
    int command;
    double twist;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(command);
        ar & BOOST_SERIALIZATION_NVP(twist);
    }
};

}
