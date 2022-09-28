#include <visualization_converter/VisualizationConverterNodelet.h>

namespace visualization_converter {

    void VisualizationConverterNodelet::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        /// Publisher
        steeringAnglePublisher = pcreate_publisher<autominy_msgs::msg::Plot>("visualization/steering_angle", 1);
        speedPublisher = pcreate_publisher<autominy_msgs::msg::Plot>("visualization/speed", 1);
        wantedSpeedPublisher = pcreate_publisher<autominy_msgs::msg::Plot>("visualization/wanted_speed", 1);
        wantedSteeringPublisher = pcreate_publisher<autominy_msgs::msg::Plot>("visualization/wanted_steering", 1);
        voltagePublisher = pcreate_publisher<autominy_msgs::msg::Plot>("visualization/voltage", 1);

        /// Subscriber
        steeringAngleSubscriber = create_subscription<>("sensors/steering", 10, &VisualizationConverterNodelet::onSteeringAngle, this);
        speedSubscriber = create_subscription<>("sensors/speed", 10, &VisualizationConverterNodelet::onSpeed, this);
        wantedSpeedSubscriber = create_subscription<>("actuators/speed", 10, &VisualizationConverterNodelet::onWantedSpeed, this);
        wantedSteeringSubscriber = create_subscription<>("actuators/steering", 10, &VisualizationConverterNodelet::onWantedSteering, this);
        voltageSubscriber = create_subscription<>("sensors/voltage", 10, &VisualizationConverterNodelet::onVoltage, this);
    }

    void VisualizationConverterNodelet::onSpeed(const autominy_msgs::msg::SpeedConstPtr& msg) {
        autominy_msgs::msg::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;
        
        speedPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onSteeringAngle(const autominy_msgs::msg::SteeringAngleConstPtr& msg) {
        autominy_msgs::msg::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        steeringAnglePublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onWantedSpeed(const autominy_msgs::msg::SpeedCommandConstPtr& msg) {
        autominy_msgs::msg::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        wantedSpeedPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onWantedSteering(const autominy_msgs::msg::SteeringCommandConstPtr& msg) {
        autominy_msgs::msg::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        wantedSteeringPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onVoltage(const autominy_msgs::msg::VoltageConstPtr& msg) {
        autominy_msgs::msg::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        voltagePublisher.publish(plot);
    }
}
