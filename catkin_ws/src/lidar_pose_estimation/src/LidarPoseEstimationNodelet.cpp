#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_pose_estimation/LidarPoseEstimation.h>
#include <lidar_pose_estimation/LidarPoseEstimationFwd.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

namespace lidar_pose_estimation {


    class LidarPoseEstimationNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        LidarPoseEstimationNodelet() = default;

        /** Destructor.
         */
        ~LidarPoseEstimationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            estimation = std::make_unique<LidarPoseEstimation>();
            poleCloudPublisher = pnh.advertise<sensor_msgs::PointCloud2>("poles", 10);

            configServer = boost::make_shared<dynamic_reconfigure::Server<LidarPoseEstimationConfig> >(pnh);
            dynamic_reconfigure::Server<LidarPoseEstimationConfig>::CallbackType f;
            f = boost::bind(&LidarPoseEstimationNodelet::callbackReconfigure, this, _1, _2);
            configServer->setCallback(f);

            laserSubscriber = pnh.subscribe("scan", 1, &LidarPoseEstimationNodelet::onLaserScan, this, ros::TransportHints().tcpNoDelay());
            calibrationTimer = pnh.createTimer(ros::Duration(1), &LidarPoseEstimationNodelet::onCalibration, this);
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(LidarPoseEstimationConfig& config, uint32_t level) {
            estimation->setConfig(config);
            calibrationTimer.setPeriod(ros::Duration(1.0 / config.execution_frequency));
            this->config = config;
        }

        void onLaserScan(const sensor_msgs::LaserScanConstPtr msg) {
            if(estimation->processLaserScan(msg)) {
            }
        }

        void onCalibration(const ros::TimerEvent& evnt) {
            if(estimation->estimateLidarPosition(evnt)) {
            }
            poleCloudPublisher.publish(estimation->getPoles());
        }

        /// Subscriber
        ros::Subscriber laserSubscriber;
        ros::Timer calibrationTimer;

        /// Publisher
        ros::Publisher poleCloudPublisher;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<LidarPoseEstimationConfig>> configServer;
        LidarPoseEstimationConfig config;

        /// pointer to the functionality class
        LidarPoseEstimationUniquePtr estimation;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(lidar_pose_estimation::LidarPoseEstimationNodelet, nodelet::Nodelet);
