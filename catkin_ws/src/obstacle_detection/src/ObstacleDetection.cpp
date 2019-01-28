#include <obstacle_detection/ObstacleDetection.h>
#include <opencv2/imgcodecs.hpp>
#include <cv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace obstacle_detection {
    ObstacleDetection::ObstacleDetection() {
        transformedPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        croppedPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        voxelGridPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    }

    ObstacleDetection::~ObstacleDetection() = default;

    void ObstacleDetection::setConfig(ObstacleDetectionConfig& config) {
        this->config = config;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleDetection::getPointcloud(
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
            double maximumDepth) {
        depthCameraModel.fromCameraInfo(depthCameraInfo);

        auto center_x = static_cast<float>(depthCameraModel.cx());
        auto center_y = static_cast<float>(depthCameraModel.cy());

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = 0.001f;
        auto constant_x = static_cast<float>(unit_scaling / depthCameraModel.fx());
        auto constant_y = static_cast<float>(unit_scaling / depthCameraModel.fy());
        auto maximum_depth = static_cast<int>(maximumDepth * 1000);

        auto depth_row = reinterpret_cast<const uint16_t*>(&depthImage->data[0]);
        int row_step = depthImage->step / sizeof(uint16_t);

        auto pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int v = 0; v < (int) depthImage->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) depthImage->width; ++u) {
                uint16_t depth = depth_row[u];

                // Skip missing points
                if (depth == 0  || depth > 2000) {
                    continue;
                }

                pcl::PointXYZ p;
                p.x = (u - center_x) * depth * constant_x;
                p.y = (v - center_y) * depth * constant_y;
                p.z = depth * 0.001f;
                pointcloud->push_back(p);
            }
        }


        pointcloud->height = 1;
        pointcloud->width = static_cast<uint32_t>(pointcloud->points.size());
        pointcloud->is_dense = true;
        pointcloud->header.stamp = depthImage->header.stamp.toNSec() / 1000ull;
        pointcloud->header.seq = depthImage->header.seq;
        pointcloud->header.frame_id = depthImage->header.frame_id;

        return pointcloud;
    }

    bool ObstacleDetection::processImage(
            const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo) {

        auto pcl = getPointcloud(depthImage, depthCameraInfo, config.maximum_depth);
        pcl_ros::transformPointCloud("base_link", *pcl, *transformedPointCloud, tfListener);

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZ>);
        voxelGridFilter.setInputCloud(transformedPointCloud);
        voxelGridFilter.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelGridFilter.filter(*downsampled);
        std::cerr << "PointCloud after filtering has: " << downsampled->size ()  << " data points." << std::endl;

        passthroughFilter.setInputCloud(downsampled);
        passthroughFilter.setFilterFieldName ("z");
        passthroughFilter.setFilterLimits (0.02, 0.5);
        passthroughFilter.filter(*croppedPointCloud);
        std::cerr << "PointCloud after pass has: " << croppedPointCloud->size ()  << " data points." << std::endl;


        tree->setInputCloud(croppedPointCloud);
        std::vector<pcl::PointIndices> clusterIndices;
        euclideanClusterExtraction.setInputCloud(croppedPointCloud);
        euclideanClusterExtraction.setClusterTolerance(0.02); // 2cm
        euclideanClusterExtraction.setMinClusterSize(500);
        euclideanClusterExtraction.setMaxClusterSize(50000);
        euclideanClusterExtraction.setSearchMethod(tree);
        euclideanClusterExtraction.extract(clusterIndices);

        markers.markers.clear();
        visualization_msgs::Marker clear;
        clear.id = 0;
        clear.type = visualization_msgs::Marker::CUBE;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);
        int i = 1;
        for (auto &clusterIndice : clusterIndices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (int indice : clusterIndice.indices)
                cloud_cluster->points.push_back (croppedPointCloud->points[indice]);
            cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size ());
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            auto marker = markCluster(cloud_cluster, "objects", i++, 255, 0, 0);
            markers.markers.push_back(marker);
        }

        return true;
    }

    visualization_msgs::MarkerArray ObstacleDetection::getObstacleMarkers() {
        return markers;
    }

    visualization_msgs::Marker ObstacleDetection::markCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster, std::string ns ,int id, float r, float g, float b)
    {
        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;

        pcl::compute3DCentroid (*cloudCluster, centroid);
        pcl::getMinMax3D (*cloudCluster, min, max);

        uint32_t shape = visualization_msgs::Marker::CUBE;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();

        marker.ns = ns;
        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = (max[0] - min[0]);
        marker.scale.y = (max[1] - min[1]);
        marker.scale.z = (max[2] - min[2]);

        if (marker.scale.x == 0)
            marker.scale.x = 0.1;

        if (marker.scale.y == 0)
            marker.scale.y = 0.1;

        if (marker.scale.z == 0)
            marker.scale.z = 0.1;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.5;

        return marker;
    }
}
