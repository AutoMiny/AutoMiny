#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <pcl/common/transforms.h>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.hpp>


namespace stereo_camera_pose_estimation {

    struct StereoCameraPoseEstimationConfig {
        double maximum_depth = 2.0;
        int aruco_id = 0;
        double aruco_size = 0.02;
        double max_yaw = 0.785;
        double max_pitch = 0.785;
        double max_roll = 0.785;
        double idle_time = 10.0;
        double x_offset = 0.0;
        double y_offset = 0.0;
        double height_offset = 0.0;
        double yaw_offset = 0.0;
        double pitch_offset = 0.0;
        double roll_offset = 0.0;
        bool use_marker = true;
        std::string camera_frame = "camera_bottom_screw_frame";
        std::string base_link_frame = "base_link";
        std::string marker_frame = "marker";
        bool debug = true;
    };

    class StereoCameraPoseEstimationNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~StereoCameraPoseEstimationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        StereoCameraPoseEstimationNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:

        void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depth_camera_info);

        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud(
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo,
                double maximumDepth);

        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloudFloat(
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo,
                double maximumDepth);


        bool processImage(
                const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo,
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo);

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getPlaneCloud();

        sensor_msgs::msg::Image::SharedPtr getMarkerImage();

        /// subscriber
        message_filters::Subscriber<sensor_msgs::msg::Image> depthImageSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::Image> infraImageSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depthCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infraCameraInfoSubscriber;

        /// Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planeCloudPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr markerImagePublisher;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicyDepthImage;
        typedef message_filters::Synchronizer<SyncPolicyDepthImage> SynchronizerDepthImage;
        std::shared_ptr<SynchronizerDepthImage> sync;


        /// pointer to dynamic reconfigure service
        StereoCameraPoseEstimationConfig config;

        image_geometry::PinholeCameraModel depthCameraModel;
        image_geometry::PinholeCameraModel imageCameraModel;
        cv_bridge::CvImagePtr image;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        tf2_ros::StaticTransformBroadcaster tfBroadcaster;
        pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPointCloud;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::ExtractIndices<pcl::PointXYZ> extractIndicesFilter;
        rclcpp::Time lastPoseEstimationTime;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}