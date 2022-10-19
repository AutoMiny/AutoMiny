#pragma once

// C++ standard
#include <cassert>
#include <iterator>
#include <stdexcept>
#include <string>

// ROS
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

// URDF
#include <urdf/model.h>

// ros_controls
#include <controller_interface/controller_interface.hpp>
#include <control_toolbox/pid.hpp>
#include <control_toolbox/pid_ros.hpp>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
//#include <hardware_interface/joint_command_interface.hpp>

// autominy
#include "autominy_msgs/msg/steering_pwm_command.hpp"
#include "autominy_msgs/msg/steering_feedback.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/tick.hpp"
#include "autominy_msgs/msg/voltage.hpp"

namespace autominy_sim_control
{

    /**
   * \brief 
   *
   */
  class AutominySimController : public controller_interface::ControllerInterface
  {
    public:
      // public interface inherited from Controller
      CallbackReturn on_init() override;
      // public interface inherited from ControllerBase 
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
      controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
      controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  private:

      struct JointHandle
      {
          std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
          std::reference_wrapper<hardware_interface::LoanedCommandInterface> effort;
      };

      // helper functions
      void setupParamas();

      // action subscriber
      void steering_callback(autominy_msgs::msg::SteeringPWMCommand::ConstSharedPtr const &msg);
      void speed_callback(autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr const &speed);

      std::string name;///< Controller name.

      std::vector<JointHandle> joints;///< Handles to controlled joints.
      std::vector<std::string> joint_names;///< Controlled joint names.

      typedef std::shared_ptr<control_toolbox::PidROS> PidPtr;
      std::vector<PidPtr> pids;

      rclcpp::Subscription<autominy_msgs::msg::SpeedPWMCommand>::SharedPtr speed_sub;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
      rclcpp::Subscription<autominy_msgs::msg::SteeringPWMCommand>::SharedPtr steering_sub;
      rclcpp::Publisher<autominy_msgs::msg::SteeringFeedback>::SharedPtr steer_angle_pub;
      rclcpp::Publisher<autominy_msgs::msg::Tick>::SharedPtr ticks_pub;
      rclcpp::Publisher<autominy_msgs::msg::Voltage>::SharedPtr voltage_pub;


      std::string drive_rear_left_joint, drive_rear_right_joint, drive_front_left_joint, drive_front_right_joint;
      std::string steer_left_joint, steer_right_joint;
      double axe_distance;
      double wheel_distance;
      double wheel_diameter;
      double zero_steer_angle;
      double motor_distance;
      std::string steering_topic;
      std::string steering_fb_topic;
      std::string speed_topic;
      std::string ticks_topic;
      std::string voltage_topic;
      rclcpp::Time last_publish;

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