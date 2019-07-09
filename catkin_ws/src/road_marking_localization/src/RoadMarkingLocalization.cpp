#include <road_marking_localization/RoadMarkingLocalization.h>
#include <opencv2/imgcodecs.hpp>
#include <cv.hpp>
#include <pcl_ros/point_cloud.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace road_marking_localization {
    RoadMarkingLocalization::RoadMarkingLocalization() : tfListener(tfBuffer) {
        croppedCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        randomSampledCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        alignedPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        transformedCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
                (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

        // Create a TransformationEstimationLM object, and set the warp to it
        auto lmTE = new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>();
        lmTE->setWarpFunction(warp_fcn);

        transformationEstimation.reset(lmTE);

        // Pass the TransformationEstimation object to the ICP algorithm
        iterativeClosestPoint.setTransformationEstimation(transformationEstimation);

        randomSampleFilter.setSeed(static_cast<unsigned int>(rand()));

        transformationMatrix.setIdentity();
    }

    RoadMarkingLocalization::~RoadMarkingLocalization() = default;

    void RoadMarkingLocalization::setConfig(RoadMarkingLocalizationConfig& config) {
        this->config = config;

        // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
        iterativeClosestPoint.setMaxCorrespondenceDistance(config.icp_max_correspondence_distance);
        // Set the maximum number of iterations (criterion 1)
        iterativeClosestPoint.setMaximumIterations(config.icp_max_iterations);
        // Set the transformation epsilon (criterion 2)
        //iterativeClosestPoint.setTransformationEpsilon (1e-9);
        // Set the euclidean distance difference epsilon (criterion 3)
        //iterativeClosestPoint.setEuclideanFitnessEpsilon (1);
        iterativeClosestPoint.setRANSACOutlierRejectionThreshold(config.icp_RANSAC_outlier_rejection_threshold);
        iterativeClosestPoint.setRANSACIterations(config.icp_RANSAC_iterations);

        randomSampleFilter.setSample(static_cast<unsigned int>(config.icp_sample_size));

        switch(config.transformation_estimation) {
            case RoadMarkingLocalization_levenberg_marquardt: {
                boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
                        (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

                auto lmTE = new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>();
                lmTE->setWarpFunction(warp_fcn);

                transformationEstimation.reset(lmTE);
                ROS_INFO("Tranformation estimation using Levenberg Marquardt");
                break;
            }
            case RoadMarkingLocalization_center_of_mass: {
                auto teCoM = new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teCoM);
                ROS_INFO("Tranformation estimation using center of mass");
                break;
            }
            case RoadMarkingLocalization_svd: {
                auto teSVD = new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teSVD);
                ROS_INFO("Tranformation estimation using SVD");
                break;
            }

            case RoadMarkingLocalization_svd2D: {
                auto teSVD = new pcl::registration::TransformationEstimationSVD2D<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teSVD);
                ROS_INFO("Tranformation estimation using SVD 2D");
                break;
            }

            case RoadMarkingLocalization_dual_quaternion: {
                auto teDQ = new pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teDQ);
                ROS_INFO("Tranformation estimation using dual quaternion");
                break;
            }

            iterativeClosestPoint.setTransformationEstimation(transformationEstimation);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr RoadMarkingLocalization::getPointcloud(
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
            const cv::Mat& mask) {
        model.fromCameraInfo(depthCameraInfo);

        auto center_x = static_cast<float>(model.cx());
        auto center_y = static_cast<float>(model.cy());

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = 0.001f;
        auto constant_x = static_cast<float>(unit_scaling / model.fx());
        auto constant_y = static_cast<float>(unit_scaling / model.fy());

        auto depth_row = reinterpret_cast<const uint16_t*>(&depthImage->data[0]);
        int row_step = depthImage->step / sizeof(uint16_t);

        auto pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pointcloud->reserve(100000);
        for (int v = 0; v < (int) depthImage->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) depthImage->width; ++u) {
                uint16_t depth = depth_row[u];

                // Skip missing points
                if (depth == 0 || mask.at<char>(v, u) == 0) {
                    continue;
                }

                pointcloud->push_back(pcl::PointXYZ(
                        (u - center_x) * depth * constant_x, (v - center_y) * depth * constant_y, depth * 0.001f));
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

    bool RoadMarkingLocalization::processImage(
            const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo) {

        try {
            cv = cv_bridge::toCvCopy(image);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
            return false;
        }
        auto& cvImage = cv->image;

        cv::Mat cropRoi = cvImage(cv::Rect(0, 0, image->width, config.crop_top_pixels));
        cropRoi.setTo(cv::Scalar(0));
        cv::blur(cvImage, cvImage, cv::Size(config.blur_kernel_size, config.blur_kernel_size));
        cv::threshold(cvImage, cvImage, config.threshold, 255, CV_THRESH_BINARY);

        auto roadMarkerCloud = getPointcloud(depthImage, depthCameraInfo, cvImage);

        Eigen::Affine3d t;
        Eigen::Affine3d baseLinkTransform;
        try {
            tfBuffer.canTransform("map", roadMarkerCloud->header.frame_id, depthImage->header.stamp, ros::Duration(0.03));
            t = tf2::transformToEigen(tfBuffer.lookupTransform("map", roadMarkerCloud->header.frame_id, depthImage->header.stamp));
            baseLinkTransform = tf2::transformToEigen(tfBuffer.lookupTransform("map", "base_link", depthImage->header.stamp));
        } catch (tf2::TransformException& e) {
            ROS_ERROR("%s", e.what());
            return false;
        }

        pcl::transformPointCloud(*roadMarkerCloud, *transformedCloud, t);
        transformedCloud->header.frame_id = "map";

        if (mapPointCloud) {
            boxFilter.setInputCloud(transformedCloud);
            auto x = baseLinkTransform.translation().x();
            auto y = baseLinkTransform.translation().y();
            boxFilter.setMin(Eigen::Vector4f(static_cast<const float&>(x - config.x_box),
                                             static_cast<const float&>(y - config.y_box),
                                             static_cast<const float&>(config.minimum_z), 1.0));
            boxFilter.setMax(Eigen::Vector4f(static_cast<const float&>(x + config.x_box),
                                             static_cast<const float&>(y + config.y_box),
                                             static_cast<const float&>(config.maximum_z), 1.0));
            boxFilter.filter(*croppedCloud);

            if (croppedCloud->size() < config.minimum_points) {
                ROS_WARN("Not enough points for correction");
                return false;
            }

            if (croppedCloud->size() > config.icp_sample_size) {
                randomSampleFilter.setInputCloud(croppedCloud);
                randomSampleFilter.filter(*randomSampledCloud);
            } else {
                randomSampledCloud = croppedCloud;
            }

            iterativeClosestPoint.setInputSource(randomSampledCloud);
            iterativeClosestPoint.align(*alignedPointCloud);

            transformationMatrix = iterativeClosestPoint.getFinalTransformation();

            auto yaw = transformationMatrix.block<3, 3>(0, 0, 3, 3).eulerAngles(0, 1, 2)[2];
            auto xCorrection = transformationMatrix(0, 3);
            auto yCorrection = transformationMatrix(1, 3);
            if (xCorrection > config.maximum_x_correction || yCorrection > config.maximum_y_correction ||
                yaw > config.maximum_yaw_correction ||
                xCorrection < -config.maximum_x_correction || yCorrection < -config.maximum_y_correction ||
                yaw < -config.maximum_yaw_correction) {
                ROS_WARN_STREAM("Too large transformation, not doing anything" << transformationMatrix);
                transformationMatrix = transformationMatrix.setIdentity();
                return false;
            }

            auto currentPos = Eigen::Vector4f(
                    static_cast<const float&>(baseLinkTransform.translation().x()),
                    static_cast<const float&>(baseLinkTransform.translation().y()),
                    static_cast<const float&>(baseLinkTransform.translation().z()), 1);
            auto pos = transformationMatrix * currentPos;
            correctedPosition.header.stamp = depthImage->header.stamp;
            correctedPosition.header.frame_id = "map";
            correctedPosition.child_frame_id = "base_link";
            correctedPosition.pose.pose.position.x = pos[0];
            correctedPosition.pose.pose.position.y = pos[1];
            correctedPosition.pose.pose.position.z = pos[2];
            correctedPosition.pose.covariance = { 0.01, 0, 0, 0, 0, 0,
                                                  0, 0.01, 0, 0, 0, 0,
                                                  0, 0, 0.01, 0, 0, 0,
                                                  0, 0, 0, 0.01, 0, 0,
                                                  0, 0, 0, 0, 0.01, 0,
                                                  0, 0, 0, 0, 0, 0.01
            };
            tf2::Quaternion q;

            q.setRPY(0, 0, baseLinkTransform.rotation().eulerAngles(0, 1, 2)[2] + yaw);
            tf2::convert(q, correctedPosition.pose.pose.orientation);
        } else {
            return false;
        }

        return true;
    }

    void RoadMarkingLocalization::setMap(const nav_msgs::OccupancyGridConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmpMap(new pcl::PointCloud<pcl::PointXYZ>);

        tf2::Quaternion quat;
        tf2::Vector3 trans;
        tf2::convert(msg->info.origin.orientation, quat);
        tf2::convert(msg->info.origin.position, trans);
        auto t = tf2::Transform(quat, trans);
        for (int x = 0; x < msg->info.height; x++) {
            for (int y = 0; y < msg->info.width; y++) {
                if (msg->data[x * msg->info.width + y] < 50) {
                    auto p = tf2::Vector3(y * msg->info.resolution, x * msg->info.resolution, 0);
                    p = t * p;

                    tmpMap->push_back(pcl::PointXYZ(static_cast<float>(p.x() + msg->info.resolution / 2.0),
                            static_cast<float>(p.y() + msg->info.resolution / 2.0), static_cast<float>(p.z())));
                }
            }
        }
        tmpMap->height = 1;
        tmpMap->width = static_cast<uint32_t>(tmpMap->points.size());
        tmpMap->is_dense = true;
        tmpMap->header.stamp = msg->header.stamp.toNSec() / 1000ull;
        tmpMap->header.seq = msg->header.seq;
        tmpMap->header.frame_id = msg->header.frame_id;

        mapPointCloud = tmpMap;

        iterativeClosestPoint.setInputTarget(mapPointCloud);
    }

    const nav_msgs::Odometry& RoadMarkingLocalization::getCorrectedPosition() {
        return correctedPosition;
    }

    void RoadMarkingLocalization::setPosition(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        geometry_msgs::Pose tfPose = pose.pose.pose;

        geometry_msgs::TransformStamped t;
        try {
            t = tfBuffer.lookupTransform("map", pose.header.frame_id, ros::Time(0));
        } catch (tf2::TransformException& e) {
            ROS_ERROR("%s", e.what());
        }
        tf2::doTransform(tfPose, tfPose, t);

        correctedPosition.header.stamp = pose.header.stamp;
        correctedPosition.header.frame_id = "map";
        correctedPosition.child_frame_id = "base_link";

        correctedPosition.pose.pose = tfPose;

        correctedPosition.pose.covariance = {  0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0
        };
        tfBuffer.clear();
    }

    sensor_msgs::ImageConstPtr RoadMarkingLocalization::getThresholdedImage() {
        return cv->toImageMsg();
    }

    sensor_msgs::PointCloud2ConstPtr RoadMarkingLocalization::getRawPointCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*transformedCloud, *ret);

        return ret;
    }

    sensor_msgs::PointCloud2ConstPtr RoadMarkingLocalization::getCroppedPointCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*croppedCloud, *ret);

        return ret;
    }

    sensor_msgs::PointCloud2ConstPtr RoadMarkingLocalization::getRandomSampledPointCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*randomSampledCloud, *ret);

        return ret;
    }

    sensor_msgs::PointCloud2ConstPtr RoadMarkingLocalization::getAlignedPointCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*alignedPointCloud, *ret);

        return ret;
    }

    sensor_msgs::PointCloud2ConstPtr RoadMarkingLocalization::getMapPointCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*mapPointCloud, *ret);

        return ret;
    }

    std_msgs::Float64MultiArrayConstPtr RoadMarkingLocalization::getTransformationMatrix() {
        auto ret = boost::make_shared<std_msgs::Float64MultiArray>();
        tf::matrixEigenToMsg(transformationMatrix, *ret);

        return ret;
    }
}
