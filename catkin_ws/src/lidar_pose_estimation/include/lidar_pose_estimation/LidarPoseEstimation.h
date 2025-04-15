#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/common/centroid.h>
#include "pcl_ros/transforms.hpp"



namespace lidar_pose_estimation {

    struct LidarPoseEstimationConfig {
        double max_dist = 0.25;
        double execution_frequency = 1.0;
        double cluster_tolerance = 0.02;
        int min_cluster_size = 10;
        int max_cluster_size = 100;
        double max_reference_distance_deviation = 0.01;
        double roll = 0.0;
        double pitch = 0.0;
        double z = 0.0;
        double p_ref_1_x = -0.02;
        double p_ref_1_y = 0.0525;
        double p_ref_2_x = -0.02;
        double p_ref_2_y = -0.0525;
    };

    class LidarPoseEstimationNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~LidarPoseEstimationNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        LidarPoseEstimationNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:

        void onLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

        void onCalibration();

        bool processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan);

        sensor_msgs::msg::PointCloud2::ConstSharedPtr getPoles();

        bool estimateLidarPosition();

        /// Subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSubscriber;
        rclcpp::TimerBase::SharedPtr calibrationTimer;

        /// Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr poleCloudPublisher;

        /// pointer to dynamic reconfigure service
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