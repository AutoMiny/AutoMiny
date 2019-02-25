#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <road_marking_localization/RoadMarkingLocalization.h>
#include <road_marking_localization/RoadMarkingLocalizationFwd.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace road_marking_localization {


    class RoadMarkingLocalizationNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        RoadMarkingLocalizationNodelet() = default;

        /** Destructor.
         */
        ~RoadMarkingLocalizationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            localization = std::make_unique<RoadMarkingLocalization>();
            image_transport::ImageTransport it(pnh);
            odometryPublisher = pnh.advertise<nav_msgs::Odometry>("corrected_odom", 10);
            thresholdedImagePublisher = pnh.advertise<sensor_msgs::Image>("threshold", 1);
            rawPclPublisher = pnh.advertise<sensor_msgs::PointCloud2>("raw_pcl", 1);
            croppedPclPublisher = pnh.advertise<sensor_msgs::PointCloud2>("cropped_pcl", 1);
            randomSampledPclPublisher = pnh.advertise<sensor_msgs::PointCloud2>("random_sampled_pcl", 1);
            alignedPclPublisher = pnh.advertise<sensor_msgs::PointCloud2>("aligned_pcl", 1);
            mapCloudPublisher = pnh.advertise<sensor_msgs::PointCloud2>("map_pcl", 1, true);

            mapSubscriber = pnh.subscribe("map", 1, &RoadMarkingLocalizationNodelet::onMap, this);
            positionEstimateSubscriber = pnh.subscribe("initialpose", 1, &RoadMarkingLocalizationNodelet::onEstimatedPosition, this);
            configServer = boost::make_shared<dynamic_reconfigure::Server<RoadMarkingLocalizationConfig> >(pnh);
            dynamic_reconfigure::Server<RoadMarkingLocalizationConfig>::CallbackType f;
            f = boost::bind(&RoadMarkingLocalizationNodelet::callbackReconfigure, this, _1, _2);
            configServer->setCallback(f);

            infraImageSubscriber.subscribe(it, "camera/infra1/image_rect_raw", 2, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
            depthImageSubscriber.subscribe(it, "camera/depth/image_rect_raw", 2, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
            infraCameraInfoSubscriber.subscribe(pnh, "camera/infra1/camera_info", 2, ros::TransportHints().tcpNoDelay());
            depthCameraInfoSubscriber.subscribe(pnh, "camera/depth/camera_info", 2, ros::TransportHints().tcpNoDelay());

            sync = std::make_shared<SynchronizerDepthImage>(SyncPolicyDepthImage(2));
            sync->connectInput(infraImageSubscriber, infraCameraInfoSubscriber, depthImageSubscriber,
                               depthCameraInfoSubscriber);
            sync->registerCallback(boost::bind(&RoadMarkingLocalizationNodelet::onImage, this, _1, _2, _3, _4));
        }

    private:
        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(RoadMarkingLocalizationConfig& config, uint32_t level) {
            localization->setConfig(config);
            this->config = config;
        }

        void onImage(
                const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg,
                const sensor_msgs::ImageConstPtr& depth_image,
                const sensor_msgs::CameraInfoConstPtr& depth_camera_info) {

            if (localization->processImage(msg, info_msg, depth_image, depth_camera_info)) {
                odometryPublisher.publish(localization->getCorrectedPosition());

                if(config.debug) {
                    rawPclPublisher.publish(localization->getRawPointCloud());
                    croppedPclPublisher.publish(localization->getCroppedPointCloud());
                    randomSampledPclPublisher.publish(localization->getRandomSampledPointCloud());
                    alignedPclPublisher.publish(localization->getAlignedPointCloud());
                    thresholdedImagePublisher.publish(localization->getThresholdedImage());
                }
            }
        }

        void onMap(const nav_msgs::OccupancyGridConstPtr& msg) {
            localization->setMap(msg);
            mapCloudPublisher.publish(localization->getMapPointCloud());
        }

        void onEstimatedPosition(const geometry_msgs::PoseWithCovarianceStamped& msg) {
            localization->setPosition(msg);
            odometryPublisher.publish(localization->getCorrectedPosition());
        }

        /// subscriber
        image_transport::SubscriberFilter depthImageSubscriber;
        image_transport::SubscriberFilter infraImageSubscriber;
        message_filters::Subscriber<sensor_msgs::CameraInfo> depthCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::CameraInfo> infraCameraInfoSubscriber;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyDepthImage;
        typedef message_filters::Synchronizer<SyncPolicyDepthImage> SynchronizerDepthImage;
        std::shared_ptr<SynchronizerDepthImage> sync;

        /// subscriber
        ros::Subscriber mapSubscriber;
        ros::Subscriber positionEstimateSubscriber;

        /// publisher
        ros::Publisher thresholdedImagePublisher;
        ros::Publisher rawPclPublisher;
        ros::Publisher croppedPclPublisher;
        ros::Publisher randomSampledPclPublisher;
        ros::Publisher alignedPclPublisher;
        ros::Publisher mapCloudPublisher;
        ros::Publisher odometryPublisher;
        tf::TransformBroadcaster odometryBroadcaster;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<RoadMarkingLocalizationConfig>> configServer;
        RoadMarkingLocalizationConfig config;

        /// pointer to the functionality class
        RoadMarkingLocalizationUniquePtr localization;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(road_marking_localization::RoadMarkingLocalizationNodelet, nodelet::Nodelet);
