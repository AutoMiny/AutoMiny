#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/clust_marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/thread/thread.hpp>


//Eigen specific
#include <Eigen/StdVector>

// PCL specific includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

class Visualizer {
public:
    Visualizer()
        : _viewer("object segmentation")
    {
        _viewer.setBackgroundColor(0.5, 0.5, 0.5);
    }

    void run() {
        while (!_viewer.wasStopped ()) {
            boost::mutex::scoped_lock updateLock(_updateModelMutex);
            _viewer.spinOnce (100);
        }
    }

    void updateData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) {        
        boost::mutex::scoped_lock updateLock(_updateModelMutex);

        if(!_viewer.updatePointCloud(cloud_clusters, "object_clusters")) {
            _viewer.addPointCloud<pcl::PointXYZRGB>(cloud_clusters, "object_clusters");
//             _viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_clusters");
        }
        if(!_viewer.updatePointCloud(cloud_plane, "segmented_plane"))
            _viewer.addPointCloud<pcl::PointXYZ>(cloud_plane, "segmented_plane");
    }

private:
    pcl::visualization::PCLVisualizer _viewer;
    boost::mutex _updateModelMutex;
};

class CloudCallback {
public:
    CloudCallback(ros::Publisher& pub, Visualizer& vis)
        : _pub(pub)
        , _vis(vis)
    { }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (*input, *cloud);
        
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int nr_points = (int) cloud_filtered->points.size ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        while (cloud_filtered->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            
            // Write the planar inliers to disk
            pcl::PointCloud<pcl::PointXYZ> cloud_plane_now;
            extract.filter (cloud_plane_now);
            *cloud_plane = cloud_plane_now;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud_filtered = cloud_f;
        }
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        
        // pack r/g/b into rgb
        uint8_t r = 250, g = 125, b = 50;    // Example: Red color
        
        int j = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            uint8_t r = (r+25)%256, g = (g+25)%256, b = (b+25)%256;
            uint32_t color = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            float fcolor = *reinterpret_cast<float*>(&color);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                pcl::PointXYZRGB p;
                p.x = cloud_filtered->points[*pit].x;
                p.y = cloud_filtered->points[*pit].y;
                p.z = cloud_filtered->points[*pit].z;
                p.rgb = fcolor;
                cloud_clusters->points.push_back(p);
            }
            j++;
        }

        _vis.updateData(cloud_clusters, cloud_plane);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg (*cloud_clusters, output);
        output.header = input->header;
        _pub.publish (output);
    }

private:
    ros::Publisher &_pub;
    Visualizer& _vis;
};

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "obj_segment");

    //Start visualizer thread
    Visualizer vis;
    boost::thread visThread(boost::bind(&Visualizer::run, &vis));    

    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("obj_clusters", 1);
    
    // Create a ROS subscriber for the input point cloud
    CloudCallback ccb(pub, vis);
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &CloudCallback::callback, &ccb);
    if(!sub) {
        std::cerr << "Oh noes!" << std::endl;
    }

    // Spin
    ros::spin ();
    visThread.join();
}
