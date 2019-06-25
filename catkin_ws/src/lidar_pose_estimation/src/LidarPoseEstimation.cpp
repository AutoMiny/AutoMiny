#include <lidar_pose_estimation/LidarPoseEstimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2/utils.h>

namespace lidar_pose_estimation {
    LidarPoseEstimation::LidarPoseEstimation() : tfListener(tfBuffer) {
        poles = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        data = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    }

    LidarPoseEstimation::~LidarPoseEstimation() = default;

    void LidarPoseEstimation::setConfig(LidarPoseEstimationConfig& config) {
        this->config = config;
    }

    bool LidarPoseEstimation::processLaserScan(const sensor_msgs::LaserScanConstPtr& scan) {
        sensor_msgs::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>());

        projector.transformLaserScanToPointCloud("lidar_base_link", *scan, cloud, tfBuffer, config.max_dist);
        pcl::fromROSMsg(cloud, *cloudptr);

        data->header = cloudptr->header;
        *data += *cloudptr;

        return false;
    }

    sensor_msgs::PointCloud2ConstPtr LidarPoseEstimation::getPoles() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*poles, *ret);

        return ret;
    }

    bool LidarPoseEstimation::estimateLidarPosition(const ros::TimerEvent& evnt) {
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
            poles->points.emplace_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
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

            ROS_ERROR_STREAM(p1);
            ROS_ERROR_STREAM(p2);

            auto pMiddle = (p1 + p2) * 0.5;
            auto pRefMiddle = (pRef1 + pRef2) * 0.5;
            auto x = pMiddle.x() - pRefMiddle.x();
            auto y = pMiddle.y() - pRefMiddle.y();

            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "sensor_board";
            transform.child_frame_id = "lidar_base_link";
            transform.transform.translation.x = -x;
            transform.transform.translation.y = -y;
            transform.transform.translation.z = config.z;
            tf2::Quaternion q;
            geometry_msgs::Quaternion qq;
            q.setRPY(config.roll, config.pitch, yaw);
            tf2::convert(q, qq);
            transform.transform.rotation = qq;

            tfBroadcaster.sendTransform(transform);

            return true;
        }

        return false;
    }


}
