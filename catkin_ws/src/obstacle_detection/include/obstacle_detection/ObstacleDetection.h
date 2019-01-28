#pragma once

#include <obstacle_detection/ObstacleDetectionConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>

namespace obstacle_detection {

/** Dummy class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class ObstacleDetection {
    public:
        /** Constructor.
         */
        ObstacleDetection();

        /** Destructor.
         */
        virtual ~ObstacleDetection();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(ObstacleDetectionConfig& config);

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

        visualization_msgs::MarkerArray getObstacleMarkers();

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

        visualization_msgs::Marker markCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster, std::string ns ,int id, float r, float g, float b);

        /// dynamic config attribute
        ObstacleDetectionConfig config;

        image_geometry::PinholeCameraModel depthCameraModel;
        tf::TransformListener tfListener;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
        pcl::PassThrough<pcl::PointXYZ> passthroughFilter;
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelGridFilter;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtraction;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr croppedPointCloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridPointCloud;

        visualization_msgs::MarkerArray markers;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
