#pragma once

#include <road_marking_localization/RoadMarkingLocalizationConfig.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/correspondence_estimation.h>
#include <road_marking_localization/transformation_estimation_svd_2d.h>
#include <road_marking_localization/random_sample.h>
#include <road_marking_localization/crop_box.h>
#include <tf2_ros/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/Float64MultiArray.h>

namespace road_marking_localization {

/** Dummy class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class RoadMarkingLocalization {
    public:
        /** Constructor.
         */
        RoadMarkingLocalization();

        /** Destructor.
         */
        virtual ~RoadMarkingLocalization();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(RoadMarkingLocalizationConfig& config);

        /**
         * Sets the map. The occupancy grid is converted into a pointcloud with z = 0
         * @param msg Occupancy grid to use
         */
        void setMap(const nav_msgs::OccupancyGridConstPtr& msg);

        /**
         * @return Updated corrected position
         */
        const nav_msgs::Odometry& getCorrectedPosition();

        /**
         * Processes the camera image and depth image to localize using road markings. The process is divided into
         * multiple stages. At first we apply gaussian blur to the camera image and threshold the white pixels. Next,
         * the pointcloud is created by combining the thresholded camera image and the depth image. In the next step
         * the resulting pointcloud is transferred into the map frame (using the current localization estimation). In
         * the next pass the pointcloud is cropped to get rid of any points that are too far away from the car (which
         * have low accuracy) and points that are not near the ground plane (z = 0). The resulting pointcloud is randomly
         * sampled to reduce the size and is passed into ICP. The ICP algorithm performs an alignment using
         * Levenberg-Marquardt optimization for x, y and yaw. The calculated alignment is then checked for plausibility
         * and being stored as the current position.
         * @param image Camera image (infrared)
         * @param cameraInfo Camera parameters
         * @param depthImage Depth image
         * @param depthCameraInfo Depth image parameters
         * @return True if correction was found, false if correction failed.
         */
        bool processImage(
                const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
                const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo);


        /**
         * Sets the position of the car using an external estimated position (from rviz)
         * @param pose The pose to set the car to
         */
        void setPosition(const geometry_msgs::PoseWithCovarianceStamped& pose);

        /**
         * Gets the thresholded image.
         * @return Thresholded image converted to ROS sensor_msgs::ImageConstPtr
         */
        sensor_msgs::ImageConstPtr getThresholdedImage();

        /**
         * Gets the map point cloud.
         * @return Map point cloud converted to ROS sensor_msgs::PointCloud2ConstPtr
         */
        sensor_msgs::PointCloud2ConstPtr getMapPointCloud();

        /**
         * Gets the raw pointcloud after transferring into the map frame.
         * @return Raw point cloud converted to ROS sensor_msgs::PointCloud2ConstPtr
         */
        sensor_msgs::PointCloud2ConstPtr getRawPointCloud();

        /**
         * Gets the cropped point cloud.
         * @return Cropped point cloud converted to ROS sensor_msgs::PointCloud2ConstPtr
         */
        sensor_msgs::PointCloud2ConstPtr getCroppedPointCloud();

        /**
         * Gets the random sampled point cloud.
         * @return Random sampled point cloud converted to ROS sensor_msgs::PointCloud2ConstPtr
         */
        sensor_msgs::PointCloud2ConstPtr getRandomSampledPointCloud();

        /**
         * Gets the aligned point cloud.
         * @return Aligned point cloud converted to ROS sensor_msgs::PointCloud2ConstPtr
         */
        sensor_msgs::PointCloud2ConstPtr getAlignedPointCloud();

        /**
         * Gets the transformation matrix
         * @return Transformation matrix converted to Float64MultiArray
         */
        std_msgs::Float64MultiArrayConstPtr getTransformationMatrix();

    private:
        /**
         * Converts a depth image into a pointcloud.
         * @param depthImage The depth image to calculate the pointcloud from.
         * @param depthCameraInfo The camera parameters.
         * @param mask A mask to use (pixels == 0 are skipped)
         * @return Pointcloud from depth image.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud(
                const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
                const cv::Mat& mask);

        /// dynamic config attribute
        RoadMarkingLocalizationConfig config;

        image_geometry::PinholeCameraModel model;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> iterativeClosestPoint;
        nav_msgs::Odometry correctedPosition;
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
        boost::shared_ptr<pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>> transformationEstimation;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
