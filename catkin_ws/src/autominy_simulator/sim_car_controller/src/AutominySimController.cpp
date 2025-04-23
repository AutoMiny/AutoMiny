#include "AutominySimController.h"
#include <limits>
#include <memory>
#include <tf2/utils.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace autominy_sim_control
{
    namespace internal
    {
        std::string getNamespace(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh) {
            const std::string complete_ns = nh->get_namespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);
        }

        double mapRange(double a1, double a2, double b1, double b2, double s) {
            return b1 + (s-a1) * (b2-b1) / (a2-a1);
        }
    }

    controller_interface::InterfaceConfiguration AutominySimController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto & joint_name : joint_names)
        {
            conf_names.push_back(joint_name + "/" + (joint_name.find("steering") != std::string::npos ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY));
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration AutominySimController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto & joint_name : joint_names)
        {
            conf_names.push_back(joint_name + "/" + (joint_name.find("steering") != std::string::npos ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY));
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn AutominySimController::on_configure(const rclcpp_lifecycle::State &previous_state) {// ros communications
        this->steer_angle_pub = get_node()->create_publisher<autominy_msgs::msg::SteeringFeedback>(this->steering_fb_topic, 1);
        this->ticks_pub = get_node()->create_publisher<autominy_msgs::msg::Tick>(this->ticks_topic, 1);
        this->voltage_pub = get_node()->create_publisher<autominy_msgs::msg::Voltage>(this->voltage_topic, 1);

        this->steering_sub = get_node()->create_subscription<autominy_msgs::msg::SteeringPWMCommand>(this->steering_topic, 1, std::bind(&AutominySimController::steering_callback, this, std::placeholders::_1));
        this->speed_sub = get_node()->create_subscription<autominy_msgs::msg::SpeedPWMCommand>(this->speed_topic, 1, std::bind(&AutominySimController::speed_callback, this, std::placeholders::_1));


        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Initialized controller '" << name << "' with:" <<
                                                                                 "\n- Number of joints: " << joints.size());

        this->last_cmd_drive = 0.0;
        this->last_cmd_steer = 0.0;

        this->joint_names.clear();

        // Controller name
        this->name = internal::getNamespace(this->get_node());

        joint_names.emplace_back(this->get_node()->get_parameter("drive_rear_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_rear_right_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_front_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_front_right_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("steer_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("steer_right_joint").as_string());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AutominySimController::on_init() {
        // load additional parameters
        setupParamas();

        RCLCPP_INFO(get_node()->get_logger(), "Init");

        std::string robotDescriptionParam = get_node()->declare_parameter<std::string>("robot_description_param", "robot_description");
        RCLCPP_INFO(get_node()->get_logger(), "Robot description %s", robotDescriptionParam.c_str());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AutominySimController::on_activate(const rclcpp_lifecycle::State &) {
        last_publish = get_node()->now();
        acc = 0.0;

        // get the joint names
        this->joint_names.clear();

        // Controller name
        this->name = internal::getNamespace(this->get_node());
        this->motor_distance = 0;

        joint_names.emplace_back(this->get_node()->get_parameter("drive_rear_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_rear_right_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_front_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("drive_front_right_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("steer_left_joint").as_string());
        joint_names.emplace_back(this->get_node()->get_parameter("steer_right_joint").as_string());

        const unsigned int n_joints = joint_names.size();
        RCLCPP_INFO(get_node()->get_logger(), "Joint-Names %d", n_joints);
        RCLCPP_INFO(get_node()->get_logger(), "Joint-Names %d", n_joints);
        RCLCPP_INFO(get_node()->get_logger(), "ROOT NS %s", get_node()->get_namespace());
        RCLCPP_INFO(get_node()->get_logger(), "Controller NS %s", get_node()->get_namespace());

        // Initialize the joints
        this->joints.clear();
        this->pids.resize(n_joints);

        for (unsigned int i = 0; i < n_joints; ++i) {
            // Joint handle
            const auto state_handle = std::find_if(
                    state_interfaces_.cbegin(), state_interfaces_.cend(),
                    [this, i](const auto & interface)
                    {
                        return interface.get_prefix_name() == this->joint_names[i];
                    });

            if (state_handle == state_interfaces_.cend())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", this->joint_names[i].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto command_handle = std::find_if(
                    command_interfaces_.begin(), command_interfaces_.end(),
                    [this, i](const auto & interface)
                    {
                        return interface.get_prefix_name() == this->joint_names[i] &&
                               interface.get_interface_name() == (this->joint_names[i].find("steering") != std::string::npos ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY);
                    });

            if (command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", this->joint_names[i].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            // Node handle to PID gains
            // Init PID gains from ROS parameter server
            this->pids[i] = std::make_shared<control_toolbox::PidROS>(get_node(), joint_names[i]);
            if(!this->pids[i]->initPid()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not initialize PID %s", joint_names[i].c_str());
                return CallbackReturn::ERROR;
            }

            this->joints.emplace_back(
                    JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
        }



        // Reset PIDs, zero effort commands
        for (unsigned int i = 0; i < this->pids.size(); ++i) {
            this->pids[i]->reset();
            if (!this->joints[i].effort.get().set_value(0.0)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %d", i);
            }
        }
        this->left_steer_cmd = 0;
        this->right_steer_cmd = 0;
        this->left_drive_cmd = 0;
        this->right_drive_cmd = 0;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AutominySimController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type AutominySimController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        //RCLCPP_INFO(get_logger(), "Duration %f", period.toSec());
        double error, command;
        double steer_l_pos, steer_r_pos;
        double drive_r_l_vel, drive_r_r_vel, drive_f_l_vel, drive_f_r_vel;

        //RCLCPP_ERROR(get_node()->get_logger(), "Cmd front right steering: %f", this->right_steer_cmd);
        //RCLCPP_ERROR(get_node()->get_logger(), "Feedback front right steering: %f", steer_r_pos);
        //RCLCPP_ERROR(get_node()->get_logger(), "Error front right steering: %f", error);
        //RCLCPP_ERROR(get_node()->get_logger(), "Front right steering: %f", command);

        /*RCLCPP_ERROR(get_node()->get_logger(), "Cmd front right speed: %f", this->right_drive_cmd);
        RCLCPP_ERROR(get_node()->get_logger(), "Feedback front right speed: %f", drive_f_r_vel);
        RCLCPP_ERROR(get_node()->get_logger(), "Error front right speed: %f", error);
        RCLCPP_ERROR(get_node()->get_logger(), "Front right speed: %f", command);*/

        if (time - last_publish >= rclcpp::Duration::from_seconds(0.01)) {
            steer_l_pos = this->joints[4].feedback.get().get_value();
            steer_r_pos = this->joints[5].feedback.get().get_value();

            drive_r_l_vel = this->joints[0].feedback.get().get_value();
            drive_r_r_vel = this->joints[1].feedback.get().get_value();
            drive_f_l_vel = this->joints[2].feedback.get().get_value();
            drive_f_r_vel = this->joints[3].feedback.get().get_value();

            double currentLinearSpeed = (drive_r_l_vel + drive_r_r_vel) / 2.0 * (this->wheel_diameter / 2.0);
            double wantedLinearSpeed = currentLinearSpeed + (this->linear_speed - currentLinearSpeed) * 0.015;

            double radius = 0.0;
            if (fabs(this->last_cmd_steer) > 0.001)
                radius = fabs(this->axe_distance / tan(this->last_cmd_steer));
            else
                radius = std::numeric_limits<double>::max();

            if (this->last_cmd_steer >= 0) {
                this->right_drive_cmd = (wantedLinearSpeed * ((radius + this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
                this->left_drive_cmd = (wantedLinearSpeed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
            } else {
                this->right_drive_cmd = (wantedLinearSpeed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
                this->left_drive_cmd = (wantedLinearSpeed * ((radius + this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
            }

            double wantedWheelSpeed = wantedLinearSpeed / (wheel_diameter / 2.0);

            // Set steering
            error = this->left_steer_cmd - steer_l_pos;
            command = std::clamp(steer_l_pos + pids[4]->computeCommand(error, (time - last_publish)), -0.57, 0.57);
            if (!this->joints[4].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[4].c_str());
            }

            error = this->right_steer_cmd - steer_r_pos;
            command = std::clamp(steer_r_pos + pids[5]->computeCommand(error, (time - last_publish)), -0.57, 0.57);
            if (!this->joints[5].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[5].c_str());
            }

            // Set Speed m/s => rad/s
            error = this->linear_speed / (wheel_diameter / 2.0) - drive_r_l_vel;
            command = wantedWheelSpeed;
            if (std::abs(linear_speed) < 0.01 && std::abs(drive_r_r_vel + drive_r_l_vel) / 2.0 < 0.01) command = 0;
            if (!this->joints[0].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[0].c_str());
            }

            error = this->linear_speed / (wheel_diameter / 2.0) - drive_r_r_vel;
            command = wantedWheelSpeed;
            if (std::abs(linear_speed) < 0.01 && std::abs(drive_r_r_vel + drive_r_l_vel) / 2.0 < 0.01) command = 0;
            if (!this->joints[1].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[1].c_str());
            }

            //RCLCPP_ERROR(get_node()->get_logger(), "lin: %f feed: %f command: %f", linear_speed, drive_r_l_vel, wheelSpeed);

            error = this->left_drive_cmd - drive_f_l_vel;
            command = left_drive_cmd;
            if (std::abs(linear_speed) < 0.01 && std::abs(drive_r_r_vel + drive_r_l_vel) / 2.0 < 0.01) command = 0;
            if (!this->joints[2].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[2].c_str());
            }

            //RCLCPP_ERROR(get_node()->get_logger(), "Vel left: %f", command);

            error = this->right_drive_cmd - drive_f_r_vel;
            command = right_drive_cmd;
            if (std::abs(linear_speed) < 0.01 && std::abs(drive_r_r_vel + drive_r_l_vel) / 2.0 < 0.01) command = 0;
            if (!this->joints[3].effort.get().set_value(command)) {
                RCLCPP_ERROR(get_node()->get_logger(), "Could not set value for joint %s", this->joint_names[3].c_str());
            }

            // Publish steer angle
            double cotan_steer, steer_angle_radians;
            cotan_steer = (1 / tan(steer_l_pos) + 1 / tan(steer_r_pos)) / 2;
            steer_angle_radians = atan2(1.0, cotan_steer);
            if (steer_angle_radians > 3.14159 / 2.0) {
                steer_angle_radians -= 3.14159;
            }

            // Publish steer angle feedback
            auto steer_angle = internal::mapRange(-0.50, 0.50, 193, 419, -steer_angle_radians);
            autominy_msgs::msg::SteeringFeedback msg;
            msg.value = static_cast<int16_t>(steer_angle);
            msg.header.stamp = time;
            msg.header.frame_id = "base_link";
            this->steer_angle_pub->publish(msg);

            // Publish ticks
            // rad / s * m/s = > rad (wheel) => * 12.5 ratio for motor,
            motor_distance += std::abs(drive_r_r_vel + drive_r_l_vel) / 2.0 * (time - last_publish).seconds() * 12.5;
            //RCLCPP_ERROR(get_node()->get_logger(), "%f %f %f %f %f %f", drive_r_r_vel, drive_r_l_vel, drive_f_r_vel, drive_f_l_vel, this->left_drive_cmd, this->right_drive_cmd);
            uint8_t wheel_ticks = std::floor(motor_distance / (M_PI / 3.0));
            motor_distance = std::fmod(motor_distance, (M_PI / 3.0));

            autominy_msgs::msg::Tick tick;
            tick.header.stamp = time;
            tick.header.frame_id = "base_link";
            tick.value = wheel_ticks;
            ticks_pub->publish(tick);

            // Publish voltage
            autominy_msgs::msg::Voltage volt;
            volt.value = 16.0f;
            volt.header.stamp = time;
            volt.header.frame_id = "base_link";
            voltage_pub->publish(volt);
            last_publish = time;
        }

        return controller_interface::return_type::OK;
    }

    void AutominySimController::setupParamas() {
        // get the car parameters
        this->axe_distance = this->auto_declare<double>("axel_distance", 0.26);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Axl distance set to: " << this->axe_distance);

        this->wheel_distance = this->auto_declare<double>("wheel_distance", 0.165);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Wheel distance set to: " << this->wheel_distance);

        this->wheel_diameter = this->auto_declare<double>("wheel_diameter", 0.063);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Wheel diameter set to: " << this->wheel_diameter);

        this->zero_steer_angle = this->auto_declare<double>("zero_steer_angle", 0.0);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Zero steer angle set to: " << this->zero_steer_angle);

        this->steering_topic = this->auto_declare<std::string>("steering_topic", "/actuators/steering_pwm");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steering topic set to: " << this->steering_topic);

        this->steering_fb_topic = this->auto_declare<std::string>("steering_fb_topic", "/steering_angle");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steering feedback topic set to: " << this->steering_fb_topic);

        this->speed_topic = this->auto_declare<std::string>("speed_topic", "/actuators/speed_pwm");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Speed topic set to: " << this->speed_topic);

        this->ticks_topic = this->auto_declare<std::string>("ticks_topic", "/ticks");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Ticks topic set to: " << this->ticks_topic);

        this->voltage_topic = this->auto_declare<std::string>("voltage_topic", "/voltage");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Voltage topic set to: " << this->voltage_topic);

        this->drive_rear_left_joint = this->auto_declare<std::string>("drive_rear_left_joint", "rearwheel_left");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Rear left joint set to: " << this->drive_rear_left_joint);

        this->drive_rear_right_joint = this->auto_declare<std::string>("drive_rear_right_joint", "rearwheel_right");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Rear right joint set to: " << this->drive_rear_right_joint);

        this->drive_front_left_joint = this->auto_declare<std::string>("drive_front_left_joint", "frontwheel_left");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Front left joint set to: " << this->drive_front_left_joint);

        this->drive_front_right_joint = this->auto_declare<std::string>("drive_front_right_joint", "frontwheel_right");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Front right joint set to: " << this->drive_front_right_joint);

        this->steer_left_joint = this->auto_declare<std::string>("steer_left_joint", "frontwheel_steering_left");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steer left joint set to: " << this->steer_left_joint);

        this->steer_right_joint = this->auto_declare<std::string>("steer_right_joint", "frontwheel_steering_right");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steer right joint set to: " << this->steer_right_joint);
    }

    void AutominySimController::steering_callback(autominy_msgs::msg::SteeringPWMCommand::ConstSharedPtr const &msg) {
        int16_t steering_command;
        double car_angle;
        double curve_radius;

        steering_command = msg->value;
        if (steering_command < 1558)
            car_angle = internal::mapRange(950, 1558, 0.512, 0, steering_command);
        else
            car_angle = internal::mapRange(2150, 1558, -0.498, 0, steering_command);
      
        if (fabs(car_angle) > 0.001)
            curve_radius = fabs(this->axe_distance / tan(car_angle));
        else
            curve_radius = std::numeric_limits<double>::max();

        //std::cout << "Car: " << car_angle << ", Car (deg): "<< (car_angle / 3.14159 * 180) << ", Radius: " << curve_radius << std::endl;

        if (car_angle > 0) {
            this->right_steer_cmd = atan2(this->axe_distance, curve_radius + this->wheel_distance / 2.0);
            this->left_steer_cmd = atan2(this->axe_distance, curve_radius - this->wheel_distance / 2.0);
        } else {
            this->right_steer_cmd = -atan2(this->axe_distance, curve_radius - this->wheel_distance / 2.0);
            this->left_steer_cmd = -atan2(this->axe_distance, curve_radius + this->wheel_distance / 2.0);
        }

        this->last_cmd_steer = car_angle;
    }

    void AutominySimController::speed_callback(autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr const &speed) {
        double radius;
        double motor_voltage, motor_speed, wheel_speed;

        if (fabs(this->last_cmd_steer) > 0.001)
            radius = fabs(this->axe_distance / tan(this->last_cmd_steer));
        else
            radius = std::numeric_limits<double>::max();

        /* convert input data [-1023 <-> 1023] */
        this->linear_speed = (speed->value - 25.0) / 1024.0 * 19000.0 / 60.0 * 6.0 * 0.003;
        if (std::abs(speed->value) < 20) {
            this->linear_speed = 0;
        }

        //std::cout << "Volt: " << motor_voltage << ", Motor RPM: " << motor_speed << ", Wheel RPM: " << wheel_speed << ", Speed (m/s): " << linear_speed << std::endl;
        this->right_drive_cmd = (linear_speed * ((radius + this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->left_drive_cmd = (linear_speed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->last_cmd_drive = linear_speed;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autominy_sim_control::AutominySimController, controller_interface::ControllerInterface)