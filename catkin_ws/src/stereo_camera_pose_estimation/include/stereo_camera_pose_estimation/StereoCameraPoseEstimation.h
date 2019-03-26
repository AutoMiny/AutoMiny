#pragma once

#include <stereo_camera_pose_estimation/StereoCameraPoseEstimationConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>

namespace stereo_camera_pose_estimation {

/** Dummy class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class StereoCameraPoseEstimation {
    public:
        /** Constructor.
         */
        StereoCameraPoseEstimation();

        /** Destructor.
         */
        virtual ~StereoCameraPoseEstimation();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(StereoCameraPoseEstimationConfig& config);

        /**
         * @param image Camera image (color)
         * @param cameraInfo Camera parameters
         * @param depthImage Depth image
         * @param depthCameraInfo Depth image parameters
         * @return True if camera pose was found, false if pose estimation failed.
         */
        bool processImage(
                const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
                const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);

        /**
         * Gets the plane pointcloud
         * @return Pointcloud message
         */
        sensor_msgs::PointCloud2ConstPtr getPlaneCloud();

        /**
         * Gets the detected markers in the camera image
         * @return Image message
         */
        sensor_msgs::ImagePtr getMarkerImage();

    private:
        /**
         * Converts a depth image into a pointcloud.
         * @param depthImage The depth image to calculate the pointcloud from.
         * @param depthCameraInfo The camera parameters.
         * @param maximumDepth Maximum depth (if depth > maximum depth point is omitted)
         * @return Pointcloud from depth image.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud(
                const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
                double maximumDepth);

        /// dynamic config attribute
        StereoCameraPoseEstimationConfig config;

        image_geometry::PinholeCameraModel depthCameraModel;
        image_geometry::PinholeCameraModel imageCameraModel;
        cv_bridge::CvImagePtr image;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        tf::TransformListener tfListener;
        tf::TransformBroadcaster tfBroadcaster;
        pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPointCloud;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::ExtractIndices<pcl::PointXYZ> extractIndicesFilter;
        ros::Time lastPoseEstimationTime;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
