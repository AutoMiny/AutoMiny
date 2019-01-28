#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <obstacle_detection/ObstacleDetection.h>
#include <obstacle_detection/ObstacleDetectionFwd.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace obstacle_detection {


    class ObstacleDetectionNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        ObstacleDetectionNodelet() = default;

        /** Destructor.
         */
        ~ObstacleDetectionNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            obstacleDetection = std::make_unique<ObstacleDetection>();
            image_transport::ImageTransport it(pnh);
            configServer = boost::make_shared<dynamic_reconfigure::Server<ObstacleDetectionConfig> >(pnh);
            dynamic_reconfigure::Server<ObstacleDetectionConfig>::CallbackType f;
            f = boost::bind(&ObstacleDetectionNodelet::callbackReconfigure, this, _1, _2);
            configServer->setCallback(f);

            markerPublisher = pnh.advertise<visualization_msgs::MarkerArray>("marker", 1);

            infraImageSubscriber.subscribe(it, "/sensors/camera/color/image_rect_color", 2);
            depthImageSubscriber.subscribe(it, "/sensors/camera/depth/image_rect_raw", 2);
            infraCameraInfoSubscriber.subscribe(pnh, "/sensors/camera/color/camera_info", 2);
            depthCameraInfoSubscriber.subscribe(pnh, "/sensors/camera/depth/camera_info", 2);

            sync = std::make_shared<SynchronizerDepthImage>(SyncPolicyDepthImage(20));
            sync->connectInput(infraImageSubscriber, infraCameraInfoSubscriber, depthImageSubscriber,
                               depthCameraInfoSubscriber);
            sync->registerCallback(boost::bind(&ObstacleDetectionNodelet::onImage, this, _1, _2, _3, _4));
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(ObstacleDetectionConfig& config, uint32_t level) {
            obstacleDetection->setConfig(config);
            this->config = config;
        }

        void onImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg,
                     const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& depth_camera_info) {

            if (obstacleDetection->processImage(msg, info_msg, depth_image, depth_camera_info) && config.debug) {
                markerPublisher.publish(obstacleDetection->getObstacleMarkers());
            }
        }

        /// subscriber
        image_transport::SubscriberFilter depthImageSubscriber;
        image_transport::SubscriberFilter infraImageSubscriber;
        message_filters::Subscriber<sensor_msgs::CameraInfo> depthCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::CameraInfo> infraCameraInfoSubscriber;

        /// Publisher
        ros::Publisher planeCloudPublisher;
        ros::Publisher markerPublisher;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyDepthImage;
        typedef message_filters::Synchronizer<SyncPolicyDepthImage> SynchronizerDepthImage;
        std::shared_ptr<SynchronizerDepthImage> sync;


        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<ObstacleDetectionConfig>> configServer;
        ObstacleDetectionConfig config;

        /// pointer to the functionality class
        ObstacleDetectionUniquePtr obstacleDetection;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(obstacle_detection::ObstacleDetectionNodelet, nodelet::Nodelet);
