#include "lidar_pose_estimation/LidarPoseEstimation.h"

namespace lidar_pose_estimation {
    LidarPoseEstimationNodelet::LidarPoseEstimationNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("lidar_pose_estimation", opts),
                                                                                              tfBroadcaster(this), tfBuffer(get_clock()), tfListener(tfBuffer) {
        poles = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud< pcl::PointXYZ>);
        data = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud< pcl::PointXYZ>);

        config.max_dist = declare_parameter<double>("maxdist", 0.25);
        config.execution_frequency = declare_parameter<double>("execution_frequency", 1.0);
        config.cluster_tolerance = declare_parameter<double>("cluster_tolerance", 0.02);
        config.min_cluster_size = declare_parameter<int>("min_cluster_size", 10);
        config.max_cluster_size = declare_parameter<int>("max_cluster_size", 100);
        config.max_reference_distance_deviation = declare_parameter<double>("max_reference_distance_deviation", 0.01);
        config.roll = declare_parameter<double>("roll", 0.0);
        config.pitch = declare_parameter<double>("pitch", 0.0);
        config.z = declare_parameter<double>("z", 0.0);
        config.p_ref_1_x = declare_parameter<double>("p_ref_1_x", -0.02);
        config.p_ref_1_y = declare_parameter<double>("p_ref_1_y", 0.0525);
        config.p_ref_2_x = declare_parameter<double>("p_ref_2_x", -0.02);
        config.p_ref_2_y = declare_parameter<double>("p_ref_2_y", -0.0525);;

        poleCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("poles", 10);

        laserSubscriber = create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&LidarPoseEstimationNodelet::onLaserScan, this, std::placeholders::_1));
        calibrationTimer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / config.execution_frequency), std::bind(&LidarPoseEstimationNodelet::onCalibration, this));
    }

    void LidarPoseEstimationNodelet::onLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        processLaserScan(msg);
    }

    void LidarPoseEstimationNodelet::onCalibration() {
        estimateLidarPosition();
        poleCloudPublisher->publish(*getPoles());
    }

    bool LidarPoseEstimationNodelet::processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan) {
        sensor_msgs::msg::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>());
        try {
            projector.transformLaserScanToPointCloud("lidar_base_link", *scan, cloud, tfBuffer, config.max_dist);
        } catch(const tf2::LookupException& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
            return false;
        }
        pcl::fromROSMsg(cloud, *cloudptr);

        data->header = cloudptr->header;
        *data += *cloudptr;

        return true;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr LidarPoseEstimationNodelet::getPoles() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*poles, *ret);

        return ret;
    }

    bool LidarPoseEstimationNodelet::estimateLidarPosition() {
        poles->clear();
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(config.cluster_tolerance);
        ec.setMinClusterSize(config.min_cluster_size);
        ec.setMaxClusterSize(config.max_cluster_size);
        ec.setInputCloud(data);
        ec.extract(clusterIndices);

        for (auto& clusterIndice : clusterIndices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int indice : clusterIndice.indices)
                cluster->points.push_back(data->points[indice]);
            cluster->width = static_cast<uint32_t> (cluster->points.size());
            cluster->height = 1;
            cluster->is_dense = true;

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            poles->points.emplace_back(centroid.x(), centroid.y(), centroid.z());
        }
        poles->header.frame_id = data->header.frame_id;

        data->clear();

        if (poles->size() == 2) {
            Eigen::Vector2f p1(poles->points[0].x, poles->points[0].y);
            Eigen::Vector2f p2(poles->points[1].x, poles->points[1].y);

            //Eigen::Vector2f pRef1(-0.035, 0.055);
            //Eigen::Vector2f pRef2(-0.035, -0.055);

            Eigen::Vector2f pRef1(config.p_ref_1_x, config.p_ref_1_y);
            Eigen::Vector2f pRef2(config.p_ref_2_x, config.p_ref_2_y);


            if (std::fabs((p1 - p2).norm() - (pRef1 - pRef2).norm()) > config.max_reference_distance_deviation) {
                return false;
            }

            auto vec = p1 - p2;
            auto yaw = std::atan(vec.x() / vec.y());
            Eigen::Rotation2Df rot(yaw);
            p1 = rot * p1;
            p2 = rot * p2;

            auto pMiddle = (p1 + p2) * 0.5;
            auto pRefMiddle = (pRef1 + pRef2) * 0.5;
            auto x = pMiddle.x() - pRefMiddle.x();
            auto y = pMiddle.y() - pRefMiddle.y();

            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = now();
            transform.header.frame_id = "board";
            transform.child_frame_id = "lidar_base_link";
            transform.transform.translation.x = -x;
            transform.transform.translation.y = -y;
            transform.transform.translation.z = config.z;
            tf2::Quaternion q;
            geometry_msgs::msg::Quaternion qq;
            q.setRPY(config.roll, config.pitch, yaw);
            tf2::convert(q, qq);
            transform.transform.rotation = qq;

            tfBroadcaster.sendTransform(transform);

            return true;
        }

        return false;
    }

}