#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <stereo_camera_pose_estimation/StereoCameraPoseEstimation.h>
#include <stereo_camera_pose_estimation/StereoCameraPoseEstimationFwd.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace stereo_camera_pose_estimation {


    class StereoCameraPoseEstimationNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        StereoCameraPoseEstimationNodelet() = default;

        /** Destructor.
         */
        ~StereoCameraPoseEstimationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            estimation = std::make_unique<StereoCameraPoseEstimation>();
            image_transport::ImageTransport it(pnh);
            planeCloudPublisher = pcreate_publisher<sensor_msgs::msg::PointCloud2>("plane_pcl", 10);

            configServer = boost::make_shared<dynamic_reconfigure::Server<StereoCameraPoseEstimationConfig> >(pnh);
            dynamic_reconfigure::Server<StereoCameraPoseEstimationConfig>::CallbackType f;
            f = boost::bind(&StereoCameraPoseEstimationNodelet::callbackReconfigure, this, _1, _2);
            configServer->setCallback(f);

            markerImagePublisher = pcreate_publisher<sensor_msgs::msg::Image>("marker", 1);

            infraImageSubscriber.subscribe(it, "camera/color/image_rect_color", 2, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
            depthImageSubscriber.subscribe(it, "camera/depth/image_rect_raw", 2, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
            infraCameraInfoSubscriber.subscribe(pnh, "camera/color/camera_info", 2, ros::TransportHints().tcpNoDelay());
            depthCameraInfoSubscriber.subscribe(pnh, "camera/depth/camera_info", 2, ros::TransportHints().tcpNoDelay());

            sync = std::make_shared<SynchronizerDepthImage>(SyncPolicyDepthImage(20));
            sync->connectInput(infraImageSubscriber, infraCameraInfoSubscriber, depthImageSubscriber,
                               depthCameraInfoSubscriber);
            sync->registerCallback(boost::bind(&StereoCameraPoseEstimationNodelet::onImage, this, _1, _2, _3, _4));
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(StereoCameraPoseEstimationConfig& config, uint32_t level) {
            estimation->setConfig(config);
            this->config = config;
        }

        void onImage(const sensor_msgs::msg::ImageConstPtr& msg, const sensor_msgs::msg::CameraInfoConstPtr& info_msg,
                     const sensor_msgs::msg::ImageConstPtr& depth_image, const sensor_msgs::msg::CameraInfoConstPtr& depth_camera_info) {

            if (estimation->processImage(msg, info_msg, depth_image, depth_camera_info) && config.debug) {
                planeCloudPublisher.publish(estimation->getPlaneCloud());
                if(config.use_marker) {
                    markerImagePublisher.publish(estimation->getMarkerImage());
                }
            }
        }

        /// subscriber
        image_transport::SubscriberFilter depthImageSubscriber;
        image_transport::SubscriberFilter infraImageSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depthCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infraCameraInfoSubscriber;

        /// Publisher
        rclcpp::Publisher<>::SharedPtr planeCloudPublisher;
        rclcpp::Publisher<>::SharedPtr markerImagePublisher;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicyDepthImage;
        typedef message_filters::Synchronizer<SyncPolicyDepthImage> SynchronizerDepthImage;
        std::shared_ptr<SynchronizerDepthImage> sync;


        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<StereoCameraPoseEstimationConfig>> configServer;
        StereoCameraPoseEstimationConfig config;

        /// pointer to the functionality class
        StereoCameraPoseEstimationUniquePtr estimation;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stereo_camera_pose_estimation::StereoCameraPoseEstimationNodelet, nodelet::Nodelet);
