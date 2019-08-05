#include "AutominySimController.h"
#include <limits>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <tf/tf.h>

namespace autominy_sim_control
{
    namespace internal
    {

        std::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh, const std::string& param_name) {
            std::shared_ptr<urdf::Model> urdf(new urdf::Model);
            std::string urdf_str;
      
            if (nh.getParam(param_name, urdf_str)) {
                if (!urdf->initString(urdf_str)) {
                    ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter (namespace: " << nh.getNamespace() << ").");
                    return std::shared_ptr<urdf::Model>();
                }
            } else if (!urdf->initParam("robot_description")) {
                ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name << "' parameter");
                return std::shared_ptr<urdf::Model>();
            }
            return urdf;
        }

        typedef std::shared_ptr<const urdf::Joint> UrdfJointConstPtr;

        std::vector<UrdfJointConstPtr> getUrdfJoints(const urdf::Model& urdf, const std::vector<std::string>& joint_names) {
            std::vector<UrdfJointConstPtr> out;
            for (unsigned int i = 0; i < joint_names.size(); ++i) {
                UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
                if (urdf_joint) {
                    out.push_back(urdf_joint);
                } else {
                    ROS_ERROR_STREAM("Could not find joint '" << joint_names[i] << "' in URDF model.");
                    return std::vector<UrdfJointConstPtr>();
                }
            }
            return out;
        }

        std::string getNamespace(const ros::NodeHandle& nh) {
            const std::string complete_ns = nh.getNamespace();
            std::size_t id   = complete_ns.find_last_of("/");
            return complete_ns.substr(id + 1);
        }

        double mapRange(double a1, double a2, double b1, double b2, double s) {
            return b1 + (s-a1) * (b2-b1) / (a2-a1);
        }
    }

    template <class HardwareInterface>
    AutominySimController<HardwareInterface>::AutominySimController() {}

    template <class HardwareInterface>
    bool AutominySimController<HardwareInterface>::init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        ROS_INFO("Init");
        std::string joint_name;

        // Cache controller node handle
        this->controller_nh = controller_nh;

        // Controller name
        this->name = internal::getNamespace(this->controller_nh);

        last_publish = ros::Time::now();
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
            return false;
        }
        const unsigned int n_joints = joint_names.size();
        ROS_INFO("Joint-Names %d", n_joints);

        // get the joints from the urdf file
        std::shared_ptr<urdf::Model> urdf = internal::getUrdf(root_nh, "robot_description");
        if (!urdf) {
            ROS_ERROR_STREAM_NAMED(name, "No robot description found");
            return false;
        }

        std::vector<internal::UrdfJointConstPtr> urdf_joints = internal::getUrdfJoints(*urdf, this->joint_names);
        if (urdf_joints.empty()) {
            ROS_ERROR_STREAM_NAMED(name, "Desired joint not found in the robot model");
            return false;
        }

        // Initialize the joints
        this->joints.resize(n_joints);
        this->pids.resize(n_joints);
        for (unsigned int i = 0; i < n_joints; ++i) {
            // Joint handle
            try {
                this->joints[i] = hw->getHandle(this->joint_names[i]);
          
                // Node handle to PID gains
                ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + this->joints[i].getName());
                // Init PID gains from ROS parameter server
                this->pids[i].reset(new control_toolbox::Pid());
                if (!this->pids[i]->init(joint_nh)) {
                    ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
                    return false;
                }
            } catch (...) {
                ROS_ERROR_STREAM_NAMED(name, "Could not find joint '" << this->joint_names[i] << "' in '" <<
                this->getHardwareInterfaceType() << "'.");
                return false;
            }
        }

        // load additional parameters
        setupParamas();

        // ros communications
        this->steering_sub = root_nh.subscribe(this->steering_topic, 1, &AutominySimController<HardwareInterface>::steering_callback, this);
        this->speed_sub = root_nh.subscribe(this->speed_topic, 1, &AutominySimController<HardwareInterface>::speed_callback, this);

        this->steer_angle_pub = root_nh.advertise<autominy_msgs::SteeringFeedback>(this->steering_fb_topic, 1);
        this->ticks_pub = root_nh.advertise<autominy_msgs::Tick>(this->ticks_topic, 1);

        this->voltage_pub = root_nh.advertise<autominy_msgs::Voltage>(this->voltage_topic, 1);

        ROS_DEBUG_STREAM_NAMED(name, "Initialized controller '" << name << "' with:" <<
        "\n- Number of joints: " << joints.size() <<
        "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'");

        this->last_cmd_drive = 0.0;
        this->last_cmd_steer = 0.0;

        return true;
    }

    template <class HardwareInterface>
    inline void AutominySimController<HardwareInterface>::starting(const ros::Time& time) {
        // Reset PIDs, zero effort commands
        for (unsigned int i = 0; i < this->pids.size(); ++i) {
            this->pids[i]->reset();
            this->joints[i].setCommand(0.0);
            ROS_INFO("Joint - Pos: %f, Speed: %f", this->joints[i].getPosition(), this->joints[i].getVelocity());
        }
        this->left_steer_cmd = 0;
        this->right_steer_cmd = 0;
        this->left_drive_cmd = 0;
        this->right_drive_cmd = 0;

        ::srand(::time(NULL));
    }

    template <class HardwareInterface>
    inline void AutominySimController<HardwareInterface>::stopping(const ros::Time& time) {}

    template <class HardwareInterface>
    void AutominySimController<HardwareInterface>::update(const ros::Time& time, const ros::Duration& period) {
        //ROS_INFO("Duration %f", period.toSec());
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
        command = pids[4]->computeCommand(error, period);
        this->joints[4].setCommand(command);

        error = this->right_steer_cmd - steer_r_pos;
        command = pids[5]->computeCommand(error, period);
        this->joints[5].setCommand(command);


        // Set Speed
        error = this->left_drive_cmd - drive_r_l_vel;
        command = pids[0]->computeCommand(error, period);
        this->joints[0].setCommand(command);

        error = this->right_drive_cmd - drive_r_r_vel;
        command = pids[1]->computeCommand(error, period);
        this->joints[1].setCommand(command);

        error = this->left_drive_cmd - drive_f_l_vel;
        command = pids[2]->computeCommand(error, period);
        this->joints[2].setCommand(command);

        error = this->right_drive_cmd - drive_f_r_vel;
        command = pids[3]->computeCommand(error, period);
        this->joints[3].setCommand(command);

        acc += (this->linear_speed * period.toSec());

        if (ros::Time::now() - last_publish >= ros::Duration(0.01)) {
            // Publish steer angle
            double cotan_steer, steer_angle_radians;
            cotan_steer = (1 / tan(steer_l_pos) + 1 / tan(steer_r_pos)) / 2;
            steer_angle_radians = atan2(1.0, cotan_steer);
            if (steer_angle_radians > 3.14159 / 2.0) {
                steer_angle_radians -= 3.14159;
            }

            // Publish steer angle feedback
            auto steer_angle = internal::mapRange(-0.498, 0.512, 192, 420, steer_angle_radians);
            autominy_msgs::SteeringFeedback msg;
            msg.value = static_cast<int16_t>(steer_angle);
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "base_link";
            this->steer_angle_pub.publish(msg);

            // Publish ticks
            autominy_msgs::Tick tick;
            tick.header.stamp = ros::Time::now();
            tick.header.frame_id = "base_link";
            tick.value = acc / 0.003;
            acc = std::fmod(acc, 0.003);
            ticks_pub.publish(tick);

            // Publish voltage
            autominy_msgs::Voltage volt;
            volt.value = 16.0f;
            volt.header.stamp = ros::Time::now();
            volt.header.frame_id = "base_link";
            voltage_pub.publish(volt);
            last_publish = ros::Time::now();
        }
    }

    // helper functions
    template <class HardwareInterface>
    bool AutominySimController<HardwareInterface>::getJointName(const std::string& param_name) {
        std::string joint_name = "";
        this->controller_nh.getParam(param_name, joint_name);
        if (joint_name == "") {
            ROS_ERROR_STREAM_NAMED(this->name, param_name << " not defined");
            return false;
        }
        this->joint_names.push_back(joint_name);
        return true;
    }

    template <class HardwareInterface>
    void AutominySimController<HardwareInterface>::setupParamas() {
        // get the car parameters
        this->axe_distance = 0.26;
        this->controller_nh.getParam("axel_distance", this->axe_distance);
        ROS_DEBUG_STREAM_NAMED(this->name, "Axl distance set to: " << this->axe_distance);

        this->wheel_distance = 0.165;
        this->controller_nh.getParam("wheel_distance", this->wheel_distance);
        ROS_DEBUG_STREAM_NAMED(this->name, "Wheel distance set to: " << this->wheel_distance);

        this->wheel_diameter = 0.063;
        this->controller_nh.getParam("wheel_diameter", this->wheel_diameter);
        ROS_DEBUG_STREAM_NAMED(this->name, "Wheel diameter set to: " << this->wheel_diameter);

        this->zero_steer_angle = 0.0;
        this->controller_nh.getParam("zero_steer_angle", this->zero_steer_angle);
        ROS_DEBUG_STREAM_NAMED(this->name, "Zero steer angle set to: " << this->zero_steer_angle);

        this->steering_topic = "/steering";
        this->controller_nh.getParam("steering_topic", this->steering_topic);
        ROS_DEBUG_STREAM_NAMED(this->name, "Steering topic set to: " << this->steering_topic);

        this->steering_fb_topic = "/steering_angle";
        this->controller_nh.getParam("steering_fb_topic", this->steering_fb_topic);
        ROS_DEBUG_STREAM_NAMED(this->name, "Steering feedback topic set to: " << this->steering_fb_topic);

        this->speed_topic = "/speed";
        this->controller_nh.getParam("speed_topic", this->speed_topic);
        ROS_DEBUG_STREAM_NAMED(this->name, "Speed topic set to: " << this->speed_topic);

        this->ticks_topic = "/ticks";
        this->controller_nh.getParam("ticks_topic", this->ticks_topic);
        ROS_DEBUG_STREAM_NAMED(this->name, "Ticks topic set to: " << this->ticks_topic);

        this->voltage_topic = "/voltage";
        this->controller_nh.getParam("voltage_topic", this->voltage_topic);
        ROS_DEBUG_STREAM_NAMED(this->name, "Voltage topic set to: " << this->voltage_topic);
    }

    template <class HardwareInterface>
    void AutominySimController<HardwareInterface>::steering_callback(autominy_msgs::SteeringPWMCommandConstPtr const &msg) {
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

        std::cout << "Car: " << car_angle << ", Car (deg): "<< (car_angle / 3.14159 * 180) << ", Radius: " << curve_radius << std::endl;

        if (car_angle > 0) {
            this->right_steer_cmd = atan2(this->axe_distance, curve_radius + this->wheel_distance / 2.0);
            this->left_steer_cmd = atan2(this->axe_distance, curve_radius - this->wheel_distance / 2.0);
        } else {
            this->right_steer_cmd = -atan2(this->axe_distance, curve_radius - this->wheel_distance / 2.0);
            this->left_steer_cmd = -atan2(this->axe_distance, curve_radius + this->wheel_distance / 2.0);
        }

        this->last_cmd_steer = car_angle;
    }

    template <class HardwareInterface>
    void AutominySimController<HardwareInterface>::speed_callback(autominy_msgs::SpeedPWMCommandConstPtr const &speed) {
        double radius;
        double motor_voltage, motor_speed, wheel_speed;

        if (fabs(this->last_cmd_steer) > 0.001)
            radius = fabs(this->axe_distance / tan(this->last_cmd_steer));
        else
            radius = std::numeric_limits<double>::max();

        // TODO: this needs to be corrected

        /* convert input data [-1000 <-> 1000] */
        motor_voltage = (speed->value / 3.0) * 5.0 / 255.0;// cmd * pwm_max_voltage / pwm_number of steps
        motor_speed = motor_voltage * 1000.0; // motor_voltage * speed_controller conversion factor (1V <-> 1000 rev/min)
        wheel_speed = motor_speed / 5.5; // motor_speed / gear_ratio of the car
        this->linear_speed = wheel_speed * 3.14159 * this->wheel_diameter / 60.0; // rpm * radius * 2pi / 60

        //std::cout << "Volt: " << motor_voltage << ", Motor RPM: " << motor_speed << ", Wheel RPM: " << wheel_speed << ", Speed (m/s): " << linear_speed << std::endl;
        this->right_drive_cmd = (linear_speed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->left_drive_cmd = (linear_speed * ((radius - this->wheel_distance / 2.0) / (radius * this->wheel_diameter / 2.0)));
        this->last_cmd_drive = linear_speed;
    }

}

namespace effort_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to a \b position interface.
   */
  typedef autominy_sim_control::AutominySimController<hardware_interface::EffortJointInterface> AutominySimController;
}

PLUGINLIB_EXPORT_CLASS(effort_controllers::AutominySimController, controller_interface::ControllerBase)