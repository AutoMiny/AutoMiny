#include <visualization_converter/VisualizationConverterNodelet.h>

namespace visualization_converter {

    void VisualizationConverterNodelet::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        /// Publisher
        steeringAnglePublisher = pnh.advertise<autominy_msgs::Plot>("visualization/steering_angle", 1);
        speedPublisher = pnh.advertise<autominy_msgs::Plot>("visualization/speed", 1);
        wantedSpeedPublisher = pnh.advertise<autominy_msgs::Plot>("visualization/wanted_speed", 1);
        wantedSteeringPublisher = pnh.advertise<autominy_msgs::Plot>("visualization/wanted_steering", 1);
        voltagePublisher = pnh.advertise<autominy_msgs::Plot>("visualization/voltage", 1);

        /// Subscriber
        steeringAngleSubscriber = pnh.subscribe("sensors/steering", 10, &VisualizationConverterNodelet::onSteeringAngle, this);
        speedSubscriber = pnh.subscribe("sensors/speed", 10, &VisualizationConverterNodelet::onSpeed, this);
        wantedSpeedSubscriber = pnh.subscribe("actuators/speed", 10, &VisualizationConverterNodelet::onWantedSpeed, this);
        wantedSteeringSubscriber = pnh.subscribe("actuators/steering", 10, &VisualizationConverterNodelet::onWantedSteering, this);
        voltageSubscriber = pnh.subscribe("sensors/voltage", 10, &VisualizationConverterNodelet::onVoltage, this);
    }

    void VisualizationConverterNodelet::onSpeed(const autominy_msgs::SpeedConstPtr& msg) {
        autominy_msgs::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;
        
        speedPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onSteeringAngle(const autominy_msgs::SteeringAngleConstPtr& msg) {
        autominy_msgs::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        steeringAnglePublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onWantedSpeed(const autominy_msgs::SpeedCommandConstPtr& msg) {
        autominy_msgs::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        wantedSpeedPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onWantedSteering(const autominy_msgs::SteeringCommandConstPtr& msg) {
        autominy_msgs::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        wantedSteeringPublisher.publish(plot);
    }

    void VisualizationConverterNodelet::onVoltage(const autominy_msgs::VoltageConstPtr& msg) {
        autominy_msgs::Plot plot;
        plot.header = msg->header;
        plot.value = msg->value;

        voltagePublisher.publish(plot);
    }
}
