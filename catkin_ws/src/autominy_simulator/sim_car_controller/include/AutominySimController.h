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
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

// URDF
#include <urdf/model.h>

// ros_controls
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

// autominy
#include <autominy_msgs/SteeringPWMCommand.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/Tick.h>
#include <autominy_msgs/Voltage.h>

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
      void steering_callback(autominy_msgs::SteeringPWMCommandConstPtr const &msg);
      void speed_callback(autominy_msgs::SpeedPWMCommandConstPtr const &speed);

      ros::NodeHandle controller_nh;
      std::string name;///< Controller name.

      typedef typename HardwareInterface::ResourceHandleType JointHandle;
      std::vector<JointHandle> joints;///< Handles to controlled joints.
      std::vector<std::string> joint_names;///< Controlled joint names.

      typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
      std::vector<PidPtr> pids;

      ros::Subscriber speed_sub;
      ros::Subscriber imu_sub;
      ros::Subscriber steering_sub;
      ros::Publisher steer_angle_pub;
      ros::Publisher ticks_pub;
      ros::Publisher voltage_pub;

      double axe_distance;
      double wheel_distance;
      double wheel_diameter;
      double zero_steer_angle;
      std::string steering_topic;
      std::string steering_fb_topic;
      std::string speed_topic;
      std::string ticks_topic;
      std::string voltage_topic;

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
  };

} // namespace