#include "AutominySimController.h"
#include <limits>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <tf2/utils.h>

namespace autominy_sim_control
{
    namespace internal
    {

        std::shared_ptr<urdf::Model> getUrdf(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const std::string& param_name) {
            std::shared_ptr<urdf::Model> urdf(new urdf::Model);
            std::string urdf_str;
      
            if (nh->get_parameter(param_name, urdf_str)) {
                if (!urdf->initString(urdf_str)) {
                    RCLCPP_ERROR_STREAM(nh->get_logger(), "Failed to parse URDF contained in '" << param_name
                                                                                                << "' parameter (namespace: "
                                                                                                << nh->get_namespace()
                                                                                                << ").");
                    return std::shared_ptr<urdf::Model>();
                }
            }
            return urdf;
        }

        typedef std::shared_ptr<const urdf::Joint> UrdfJointConstPtr;

        std::vector<UrdfJointConstPtr> getUrdfJoints(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh, const urdf::Model& urdf, const std::vector<std::string>& joint_names) {
            std::vector<UrdfJointConstPtr> out;
            for (unsigned int i = 0; i < joint_names.size(); ++i) {
                UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
                if (urdf_joint) {
                    out.push_back(urdf_joint);
                } else {
                    RCLCPP_ERROR_STREAM(nh->get_logger(), "Could not find joint '" << joint_names[i] << "' in URDF model.");
                    return std::vector<UrdfJointConstPtr>();
                }
            }
            return out;
        }

        std::string getNamespace(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh) {
            const std::string complete_ns = nh->get_namespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);
        }

        double mapRange(double a1, double a2, double b1, double b2, double s) {
            return b1 + (s-a1) * (b2-b1) / (a2-a1);
        }
    }

    controller_interface::CallbackReturn AutominySimController::on_init() {
        RCLCPP_INFO(get_node()->get_logger(), "Init");
        std::string joint_name;

        // Controller name
        this->name = internal::getNamespace(this->get_node());

        last_publish = get_node()->now();
        acc = 0.0;

        // get the joint names
        this->joint_names.clear();
        if (!(getJointName("drive_rear_left_joint") &&
            getJointName("drive_rear_right_joint") &&
            getJointName("drive_front_left_joint") &&
            getJointName("drive_front_right_joint") &&
            getJointName("steer_left_joint") &&
            getJointName("steer_right_joint")))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Joints names were not found");
            return CallbackReturn::FAILURE;
        }
        const unsigned int n_joints = joint_names.size();
        RCLCPP_INFO(get_node()->get_logger(), "Joint-Names %d", n_joints);
        RCLCPP_INFO(get_node()->get_logger(), "Joint-Names %d", n_joints);
        RCLCPP_INFO(get_node()->get_logger(), "ROOT NS %s", get_node()->get_namespace());
        RCLCPP_INFO(get_node()->get_logger(), "Controller NS %s", get_node()->get_namespace());

        std::string robotDescriptionParam = get_node()->declare_parameter<std::string>("robot_description_param", "robot_description");
        RCLCPP_INFO(get_node()->get_logger(), "Robot description %s", robotDescriptionParam.c_str());

        // get the joints from the urdf file
        std::shared_ptr<urdf::Model> urdf = internal::getUrdf(get_node(), robotDescriptionParam);
        if (!urdf) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(), "No robot description found");
            return CallbackReturn::FAILURE;
        }

        std::vector<internal::UrdfJointConstPtr> urdf_joints = internal::getUrdfJoints(get_node(), *urdf, this->joint_names);
        if (urdf_joints.empty()) {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Desired joint not found in the robot model");
            return CallbackReturn::FAILURE;
        }

        // Initialize the joints
        this->joints.resize(n_joints);
        this->pids.resize(n_joints);
        for (unsigned int i = 0; i < n_joints; ++i) {
            // Joint handle
            try {
                this->joints[i] = hw->getHandle(this->joint_names[i]);
          
                // Node handle to PID gains
                // Init PID gains from ROS parameter server
                this->pids[i].reset(new control_toolbox::Pid());
                this->pids[i]->initPid(get_node()->get_parameter("gains/" + this->joints[i].getName() + "/p").as_double(),
                                       get_node()->get_parameter("gains/" + this->joints[i].getName() + "/i").as_double(),
                                       get_node()->get_parameter("gains/" + this->joints[i].getName() + "/d").as_double());
            } catch (...) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Could not find joint '" << this->joint_names[i] << "' in '" <<
                this->getHardwareInterfaceType() << "'.");
                return CallbackReturn::FAILURE;
            }
        }

        // load additional parameters
        setupParamas();

        // ros communications
        this->steering_sub = get_node()->create_subscription(this->steering_topic, 1, std::bind(&AutominySimController::steering_callback, this, std::placeholders::_1));
        this->speed_sub = get_node()->create_subscription(this->speed_topic, 1, std::bind(&AutominySimController::speed_callback, this, std::placeholders::_1));

        this->steer_angle_pub = get_node()->create_publisher<autominy_msgs::msg::SteeringFeedback>(this->steering_fb_topic, 1);
        this->ticks_pub = get_node()->create_publisher<autominy_msgs::msg::Tick>(this->ticks_topic, 1);

        this->voltage_pub = get_node()->create_publisher<autominy_msgs::msg::Voltage>(this->voltage_topic, 1);

        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Initialized controller '" << name << "' with:" <<
        "\n- Number of joints: " << joints.size());

        this->last_cmd_drive = 0.0;
        this->last_cmd_steer = 0.0;

        return CallbackReturn::SUCCESS;
    }

    inline void AutominySimController::starting(const rclcpp::Time& time) {
        // Reset PIDs, zero effort commands
        for (unsigned int i = 0; i < this->pids.size(); ++i) {
            this->pids[i]->reset();
            this->joints[i].setCommand(0.0);
            RCLCPP_INFO(get_node()->get_logger(), "Joint - Pos: %f, Speed: %f", this->joints[i].getPosition(), this->joints[i].getVelocity());
        }
        this->left_steer_cmd = 0;
        this->right_steer_cmd = 0;
        this->left_drive_cmd = 0;
        this->right_drive_cmd = 0;

        ::srand(::time(NULL));
    }

    inline void AutominySimController::stopping(const rclcpp::Time& time) {}

    controller_interface::return_type AutominySimController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        //RCLCPP_INFO(get_logger(), "Duration %f", period.toSec());
        double error, command;
        double steer_l_pos, steer_r_pos;
        double drive_r_l_vel, drive_r_r_vel, drive_f_l_vel, drive_f_r_vel;

        steer_l_pos = this->joints[4].getPosition();
        steer_r_pos = this->joints[5].getPosition();

        drive_r_l_vel = this->joints[0].getVelocity();
        drive_r_r_vel = this->joints[1].getVelocity();
        drive_f_l_vel = this->joints[2].getVelocity();
        drive_f_r_vel = this->joints[3].getVelocity();

        // Set steering
        error = this->left_steer_cmd - steer_l_pos;
        command = pids[4]->computeCommand(error, period.nanoseconds());
        this->joints[4].setCommand(command);

        error = this->right_steer_cmd - steer_r_pos;
        command = pids[5]->computeCommand(error, period.nanoseconds());
        this->joints[5].setCommand(command);


        // Set Speed
        error = this->left_drive_cmd - drive_r_l_vel;
        command = pids[0]->computeCommand(error, period.nanoseconds());
        this->joints[0].setCommand(command);

        error = this->right_drive_cmd - drive_r_r_vel;
        command = pids[1]->computeCommand(error, period.nanoseconds());
        this->joints[1].setCommand(command);

        error = this->left_drive_cmd - drive_f_l_vel;
        command = pids[2]->computeCommand(error, period.nanoseconds());
        this->joints[2].setCommand(command);

        error = this->right_drive_cmd - drive_f_r_vel;
        command = pids[3]->computeCommand(error, period.nanoseconds());
        this->joints[3].setCommand(command);

        acc += (std::abs(this->linear_speed) * period.seconds());

        if (get_node()->now() - last_publish >= rclcpp::Duration::from_seconds(0.01)) {
            // Publish steer angle
            double cotan_steer, steer_angle_radians;
            cotan_steer = (1 / tan(steer_l_pos) + 1 / tan(steer_r_pos)) / 2;
            steer_angle_radians = atan2(1.0, cotan_steer);
            if (steer_angle_radians > 3.14159 / 2.0) {
                steer_angle_radians -= 3.14159;
            }

            // Publish steer angle feedback
            auto steer_angle = internal::mapRange(-0.498, 0.512, 192, 420, -steer_angle_radians);
            autominy_msgs::msg::SteeringFeedback msg;
            msg.value = static_cast<int16_t>(steer_angle);
            msg.header.stamp = get_node()->now();
            msg.header.frame_id = "base_link";
            this->steer_angle_pub->publish(msg);

            // Publish ticks
            autominy_msgs::msg::Tick tick;
            tick.header.stamp = get_node()->now();
            tick.header.frame_id = "base_link";
            tick.value = acc / 0.003;
            acc = std::fmod(acc, 0.003);
            ticks_pub->publish(tick);

            // Publish voltage
            autominy_msgs::msg::Voltage volt;
            volt.value = 16.0f;
            volt.header.stamp = get_node()->now();
            volt.header.frame_id = "base_link";
            voltage_pub->publish(volt);
            last_publish = get_node()->now();
        }

        return controller_interface::return_type::OK;
    }

    // helper functions
    bool AutominySimController::getJointName(const std::string& param_name) {
        std::string joint_name = "";
        this->controller_nh.getParam(param_name, joint_name);
        if (joint_name == "") {
            RCLCPP_ERROR_STREAM(get_node()->get_logger(), param_name << " not defined");
            return false;
        }
        this->joint_names.push_back(joint_name);
        return true;
    }

    void AutominySimController::setupParamas() {
        // get the car parameters
        this->axe_distance = this->auto_declare("axel_distance", 0.26);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Axl distance set to: " << this->axe_distance);

        this->wheel_distance = this->auto_declare("wheel_distance", 0.165);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Wheel distance set to: " << this->wheel_distance);

        this->wheel_diameter = this->auto_declare("wheel_diameter", 0.063);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Wheel diameter set to: " << this->wheel_diameter);

        this->zero_steer_angle = this->auto_declare("zero_steer_angle", 0.0);
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Zero steer angle set to: " << this->zero_steer_angle);

        this->steering_topic = this->auto_declare("steering_topic", "/steering");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steering topic set to: " << this->steering_topic);

        this->steering_fb_topic = this->auto_declare("steering_fb_topic", "/steering_angle");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Steering feedback topic set to: " << this->steering_fb_topic);

        this->speed_topic = this->auto_declare("speed_topic", "/speed");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Speed topic set to: " << this->speed_topic);

        this->ticks_topic = this->auto_declare("ticks_topic", "/ticks");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Ticks topic set to: " << this->ticks_topic);

        this->voltage_topic = this->auto_declare("voltage_topic", "/voltage");
        RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "Voltage topic set to: " << this->voltage_topic);
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

        // TODO: this needs to be corrected

        auto val = speed->value;
        if (std::abs(val) < 20) {
            val = 0;
        }
        /* convert input data [-1000 <-> 1000] */
        motor_voltage = (val / 3.0) * 5.0 / 255.0;// cmd * pwm_max_voltage / pwm_number of steps
        motor_speed = motor_voltage * 1000.0; // motor_voltage * speed_controller conversion factor (1V <-> 1000 rev/min)
        wheel_speed = motor_speed / 5.5; // motor_speed / gear_ratio of the car
        this->linear_speed = wheel_speed * 3.14159 * this->wheel_diameter / 60.0; // rpm * radius * 2pi / 60

        //std::cout << "Volt: " << motor_voltage << ", Motor RPM: " << motor_speed << ", Wheel RPM: " << wheel_speed << ", Speed (m/s): " << linear_speed << std::endl;
        this->right_drive_cmd = (linear_speed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->left_drive_cmd = (linear_speed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->last_cmd_drive = linear_speed;
    }

}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(autominy_sim_control::AutominySimController, controller_interface::ControllerInterface)
