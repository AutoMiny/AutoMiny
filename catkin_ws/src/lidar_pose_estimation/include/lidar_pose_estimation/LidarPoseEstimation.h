#pragma once

#include <lidar_pose_estimation/LidarPoseEstimationConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

namespace lidar_pose_estimation {

/** Dummy class. Contains the general functionality of this package.
 **
 ** @ingroup @@
 */
    class LidarPoseEstimation {
    public:
        /** Constructor.
         */
        LidarPoseEstimation();

        /** Destructor.
         */
        virtual ~LidarPoseEstimation();

        /** Sets the current dynamic configuration.
         **
         ** @param config
         */
        void setConfig(LidarPoseEstimationConfig& config);

        bool processLaserScan(const sensor_msgs::msg::LaserScanConstPtr& scan);

        bool estimateLidarPosition(const ros::TimerEvent& evnt);

        sensor_msgs::msg::PointCloud2ConstPtr getPoles();

    private:
        /// dynamic config attribute
        LidarPoseEstimationConfig config;

        pcl::PointCloud<pcl::PointXYZ>::Ptr poles;
        pcl::PointCloud<pcl::PointXYZ>::Ptr data;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        tf2_ros::StaticTransformBroadcaster tfBroadcaster;
        laser_geometry::LaserProjection projector;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
