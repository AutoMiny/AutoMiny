#pragma once

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "robot_localization/srv/set_pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pcl/registration/icp.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"


namespace road_marking_localization {

    struct RoadMarkingLocalizationConfig {
        int blur_kernel_size = 1;
        int crop_top_pixels = 50;
        double x_box = 2.0;
        double y_box = 2.0;
        double minimum_z = -0.02;
        double maximum_z = 0.02;
        int threshold = 160;
        int icp_max_iterations = 5;
        double icp_RANSAC_outlier_rejection_threshold = 0.0;
        int icp_RANSAC_iterations = 0;
        double icp_max_correspondence_distance = 0.10;
        int icp_sample_size = 750;
        int minimum_points = 250;
        double maximum_x_correction = 0.3;
        double maximum_y_correction = 0.3;
        double maximum_yaw_correction = 0.5;
        bool debug = true;
        std::string base_link_frame = "base_link";
        std::string map_frame = "map";
        int transformation_estimation = 0;
    };

    class RoadMarkingLocalizationNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~RoadMarkingLocalizationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        RoadMarkingLocalizationNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:
        void onImage(
                const sensor_msgs::msg::Image::ConstSharedPtr& msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depth_camera_info);

        void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);

        void onEstimatedPosition(const geometry_msgs::msg::PoseWithCovarianceStamped msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud(
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo,
                const cv::Mat& mask);

        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloudFloat(
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo,
                const cv::Mat& mask);


        bool processImage(
                const sensor_msgs::msg::Image::ConstSharedPtr& image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo,
                const sensor_msgs::msg::Image::ConstSharedPtr& depthImage, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthCameraInfo);

        void setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);

        const nav_msgs::msg::Odometry& getCorrectedPosition() {
            return correctedPosition;
        }

        void setPosition(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);

        sensor_msgs::msg::Image::ConstSharedPtr getThresholdedImage();

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getRawPointCloud();

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getCroppedPointCloud();

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getRandomSampledPointCloud();

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getAlignedPointCloud();

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getMapPointCloud();

        void setConfig();

        /// subscriber
        image_transport::SubscriberFilter depthImageSubscriber;
        image_transport::SubscriberFilter infraImageSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depthCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infraCameraInfoSubscriber;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicyDepthImage;
        typedef message_filters::Synchronizer<SyncPolicyDepthImage> SynchronizerDepthImage;
        std::shared_ptr<SynchronizerDepthImage> sync;

        /// subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr positionEstimateSubscriber;

        /// publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thresholdedImagePublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawPclPublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr croppedPclPublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr randomSampledPclPublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr alignedPclPublisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapCloudPublisher;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
        rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr robotLocalizationSetPose;

        RoadMarkingLocalizationConfig config;

        image_geometry::PinholeCameraModel model;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> iterativeClosestPoint;
        nav_msgs::msg::Odometry correctedPosition;
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        pcl::RandomSample<pcl::PointXYZ> randomSampleFilter;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr randomSampledCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;
        cv_bridge::CvImagePtr cv;
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformationMatrix;
        std::shared_ptr<pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>> transformationEstimation;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}