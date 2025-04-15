#ifndef BNO055_USB_STICK_DECODER_HPP
#define BNO055_USB_STICK_DECODER_HPP

#include <algorithm>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/convert.h"

#include "bno055_usb_stick/constants.hpp"
#include "bno055_usb_stick_msgs/msg/calibration_status.hpp"
#include "bno055_usb_stick_msgs/msg/euler_angles.hpp"
#include "bno055_usb_stick_msgs/msg/output.hpp"

#include <boost/cstdint.hpp>

namespace bno055_usb_stick {
    class Decoder {
    public:
        Decoder(rclcpp::Node& nh)
                : nh(nh), frame_id_(nh.declare_parameter<std::string>("frame_id", "bno055")) {}

        virtual ~Decoder() {}

        bno055_usb_stick_msgs::msg::Output decode(const boost::uint8_t* data) {
            bno055_usb_stick_msgs::msg::Output output;
            output.header.stamp = nh.now();
            output.header.frame_id = frame_id_;
            output.acceleration = decodeAcc(data + Constants::ACC_POS);
            output.magnetometer = decodeMag(data + Constants::MAG_POS);
            output.gyroscope = decodeGyr(data + Constants::GYR_POS);
            output.euler_angles = decodeEul(data + Constants::EUL_POS);
            output.quaternion = decodeQua(data + Constants::QUA_POS);
            output.linear_acceleration = decodeLia(data + Constants::LIA_POS);
            output.gravity_vector = decodeGrv(data + Constants::GRV_POS);
            output.temperature = decodeTemp(data + Constants::TEMP_POS);
            output.calibration_status = decodeCalibStat(data + Constants::CALIB_STAT_POS);
            return output;
        }

        static geometry_msgs::msg::TransformStamped toTFTransform(const bno055_usb_stick_msgs::msg::Output& output,
                                                  const std::string& frame_id,
                                                  const std::string& child_frame_id,
                                                  const bool do_invert) {
            geometry_msgs::msg::Quaternion quaternion;
            geometry_msgs::msg::TransformStamped t;
            t.transform.rotation = output.quaternion;
            t.header.stamp = output.header.stamp;
            t.header.frame_id = frame_id;
            t.child_frame_id = child_frame_id;

            return t;
        }

        static sensor_msgs::msg::Imu toImuMsg(const bno055_usb_stick_msgs::msg::Output& output) {
            sensor_msgs::msg::Imu imu;
            imu.header = output.header;
            imu.orientation = output.quaternion;
            imu.angular_velocity = output.gyroscope;
            imu.linear_acceleration = output.linear_acceleration;

            // Covariances. The Bosch BNO055 datasheet is pretty useless regarding the sensor's accuracy.
            // - The accuracy of the magnetometer is +-2.5deg. Users on online forums agree on that number.
            // - The accuracy of the gyroscope is unknown. I use the +-3deg/s zero rate offset. To be tested.
            // - The accuracy of the accelerometer is unknown. Based on the typical and maximum zero-g offset (+-80mg and
            //   +-150mg) and the fact that my graphs look better than that, I use 80mg. To be tested.
            // Cross-axis errors are not (yet) taken into account. To be tested.
            for(unsigned row = 0; row < 3; ++ row) {
                for(unsigned col = 0; col < 3; ++ col) {
                    imu.orientation_covariance[row * 3 + col] = (row == col? 0.002: 0.);  // +-2.5deg
                    imu.angular_velocity_covariance[row * 3 + col] = (row == col? 0.003: 0.);  // +-3deg/s
                    imu.linear_acceleration_covariance[row * 3 + col] = (row == col? 0.60: 0.);  // +-80mg
                }
            }

            return imu;
        }

        static geometry_msgs::msg::PoseStamped toPoseMsg(const bno055_usb_stick_msgs::msg::Output& output,
                                                    const std::string& frame_id) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = output.header;
            pose.header.frame_id = frame_id;
            pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0.;
            pose.pose.orientation = output.quaternion;
            return pose;
        }

        static sensor_msgs::msg::MagneticField toMagMsg(const bno055_usb_stick_msgs::msg::Output& output) {
            sensor_msgs::msg::MagneticField mag;
            mag.header = output.header;
            mag.magnetic_field = output.magnetometer;
            std::fill(mag.magnetic_field_covariance.begin(), mag.magnetic_field_covariance.end(), 0.);
            return mag;
        }

        static sensor_msgs::msg::Temperature toTempMsg(const bno055_usb_stick_msgs::msg::Output& output) {
            sensor_msgs::msg::Temperature temp;
            temp.header = output.header;
            temp.temperature = output.temperature;
            temp.variance = 0.;
            return temp;
        }

    private:
        static geometry_msgs::msg::Vector3 decodeAcc(const boost::uint8_t* data) {
            geometry_msgs::msg::Vector3 acc;
            acc.x = decodeVal(data[1], data[0], Constants::ACC_DENOM);
            acc.y = decodeVal(data[3], data[2], Constants::ACC_DENOM);
            acc.z = decodeVal(data[5], data[4], Constants::ACC_DENOM);
            return acc;
        }

        static geometry_msgs::msg::Vector3 decodeMag(const boost::uint8_t* data) {
            geometry_msgs::msg::Vector3 mag;
            mag.x = decodeVal(data[1], data[0], Constants::MAG_DENOM);
            mag.y = decodeVal(data[3], data[2], Constants::MAG_DENOM);
            mag.z = decodeVal(data[5], data[4], Constants::MAG_DENOM);
            return mag;
        }

        static geometry_msgs::msg::Vector3 decodeGyr(const boost::uint8_t* data) {
            geometry_msgs::msg::Vector3 gyr;
            gyr.x = decodeVal(data[1], data[0], Constants::GYR_DENOM) * M_PI / 180.;
            gyr.y = decodeVal(data[3], data[2], Constants::GYR_DENOM) * M_PI / 180.;
            gyr.z = decodeVal(data[5], data[4], Constants::GYR_DENOM) * M_PI / 180.;
            return gyr;
        }

        static bno055_usb_stick_msgs::msg::EulerAngles decodeEul(const boost::uint8_t* data) {
            bno055_usb_stick_msgs::msg::EulerAngles eul;
            eul.heading = decodeVal(data[1], data[0], Constants::EUL_DENOM) * M_PI / 180.;
            eul.roll = decodeVal(data[3], data[2], Constants::EUL_DENOM) * M_PI / 180.;
            eul.pitch = decodeVal(data[5], data[4], Constants::EUL_DENOM) * M_PI / 180.;
            return eul;
        }

        static geometry_msgs::msg::Quaternion decodeQua(const boost::uint8_t* data) {
            geometry_msgs::msg::Quaternion qua;
            qua.w = decodeVal(data[1], data[0], Constants::QUA_DENOM);
            qua.x = decodeVal(data[3], data[2], Constants::QUA_DENOM);
            qua.y = decodeVal(data[5], data[4], Constants::QUA_DENOM);
            qua.z = decodeVal(data[7], data[6], Constants::QUA_DENOM);
            return qua;
        }

        static geometry_msgs::msg::Vector3 decodeLia(const boost::uint8_t* data) {
            geometry_msgs::msg::Vector3 lia;
            lia.x = decodeVal(data[1], data[0], Constants::LIA_DENOM);
            lia.y = decodeVal(data[3], data[2], Constants::LIA_DENOM);
            lia.z = decodeVal(data[5], data[4], Constants::LIA_DENOM);
            return lia;
        }

        static geometry_msgs::msg::Vector3 decodeGrv(const boost::uint8_t* data) {
            geometry_msgs::msg::Vector3 grv;
            grv.x = decodeVal(data[1], data[0], Constants::GRV_DENOM);
            grv.y = decodeVal(data[3], data[2], Constants::GRV_DENOM);
            grv.z = decodeVal(data[5], data[4], Constants::GRV_DENOM);
            return grv;
        }

        static double decodeTemp(const boost::uint8_t* data) { return data[0] / Constants::TEMP_DENOM; }

        static bno055_usb_stick_msgs::msg::CalibrationStatus decodeCalibStat(const boost::uint8_t* data) {
            bno055_usb_stick_msgs::msg::CalibrationStatus calib_stat;
            calib_stat.system = (*data >> 6) & 0x3;
            calib_stat.gyroscope = (*data >> 4) & 0x3;
            calib_stat.accelerometer = (*data >> 2) & 0x3;
            calib_stat.magnetometer = *data & 0x3;
            return calib_stat;
        }

        static double decodeVal(const boost::uint8_t msb, const boost::uint8_t lsb, const double denom) {
            return boost::int16_t((boost::int16_t(msb) << 8) | lsb) / denom;
        }

    private:
        const std::string frame_id_;
        rclcpp::Node& nh;
    };
} // namespace bno055_usb_stick
#endif // BNO055_USB_STICK_DECODER_HPP