#pragma once

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/node_handle.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

// URDF
#include <urdf/model.h>

// ros_controls
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

// autominy
#include "autominy_msgs/msg/steering_pwm_command.hpp"
#include "autominy_msgs/msg/steering_feedback.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/tick.hpp"
#include "autominy_msgs/msg/voltage.hpp"

#include <pluginlib/class_list_macros.h>

namespace autominy_sim_control
{
  /**
   * \brief 
   *
   */
  template <class HardwareInterface>
  class AutominySimController : public controller_interface::Controller<HardwareInterface>
  {
    public:
      AutominySimController();
      // public interface inherited from Controller
      bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
      // public interface inherited from ControllerBase 
      void starting(const ros::Time& time);
      void stopping(const ros::Time& time);
      void update(const ros::Time& time, const ros::Duration& period);

    private:
      // helper functions
      bool getJointName(const std::string& param_name);
      void setupParamas();

      // action subscriber
      void steering_callback(autominy_msgs::msg::SteeringPWMCommand::ConstSharedPtr const &msg);
      void speed_callback(autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr const &speed);

      ros::NodeHandle controller_nh;
      std::string name;///< Controller name.

      typedef typename HardwareInterface::ResourceHandleType JointHandle;
      std::vector<JointHandle> joints;///< Handles to controlled joints.
      std::vector<std::string> joint_names;///< Controlled joint names.

      typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
      std::vector<PidPtr> pids;

      rclcpp::Subscription<>::SharedPtr speed_sub;
      rclcpp::Subscription<>::SharedPtr imu_sub;
      rclcpp::Subscription<>::SharedPtr steering_sub;
      rclcpp::Publisher<>::SharedPtr steer_angle_pub;
      rclcpp::Publisher<>::SharedPtr ticks_pub;
      rclcpp::Publisher<>::SharedPtr voltage_pub;

      double axe_distance;
      double wheel_distance;
      double wheel_diameter;
      double zero_steer_angle;
      std::string steering_topic;
      std::string steering_fb_topic;
      std::string speed_topic;
      std::string ticks_topic;
      std::string voltage_topic;
      ros::Time last_publish;

      // joint commands
      double left_steer_cmd;
      double right_steer_cmd;

      // drive commands
      double left_drive_cmd;
      double right_drive_cmd;

      // ?
      double last_cmd_drive;
      double last_cmd_steer;
      double linear_speed;
      double acc;
  };

} // namespace