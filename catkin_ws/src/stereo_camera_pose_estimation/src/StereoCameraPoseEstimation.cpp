#include <stereo_camera_pose_estimation/StereoCameraPoseEstimation.h>
#include <opencv2/imgcodecs.hpp>
#include <cv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

namespace stereo_camera_pose_estimation {
    StereoCameraPoseEstimation::StereoCameraPoseEstimation() : tfListener(tfBuffer) {
        coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        planePointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        transformedPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    }

    StereoCameraPoseEstimation::~StereoCameraPoseEstimation() = default;

    void StereoCameraPoseEstimation::setConfig(StereoCameraPoseEstimationConfig& config) {
        this->config = config;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCameraPoseEstimation::getPointcloud(
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
                if (depth == 0  || depth > maximum_depth) {
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCameraPoseEstimation::getPointcloudFloat(
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
            double maximumDepth) {
        depthCameraModel.fromCameraInfo(depthCameraInfo);

        auto center_x = static_cast<float>(depthCameraModel.cx());
        auto center_y = static_cast<float>(depthCameraModel.cy());

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = 1.0;
        auto constant_x = static_cast<float>(unit_scaling / depthCameraModel.fx());
        auto constant_y = static_cast<float>(unit_scaling / depthCameraModel.fy());
        auto maximum_depth = maximumDepth;

        auto depth_row = reinterpret_cast<const float*>(&depthImage->data[0]);
        int row_step = depthImage->step / sizeof(float);

        auto pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int v = 0; v < (int) depthImage->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) depthImage->width; ++u) {
                auto depth = depth_row[u];

                // Skip missing points
                if (!std::isfinite(depth) || depth > maximum_depth) {
                    continue;
                }

                pcl::PointXYZ p;
                p.x = (u - center_x) * depth * constant_x;
                p.y = (v - center_y) * depth * constant_y;
                p.z = depth;
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


    bool StereoCameraPoseEstimation::processImage(
            const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
            const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo) {

        auto durationSinceLastCalibration = ros::Time::now() - lastPoseEstimationTime;
        if (durationSinceLastCalibration.toSec() < config.idle_time) {
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl;
        if (depthImage->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            pcl = getPointcloudFloat(depthImage, depthCameraInfo, config.maximum_depth);
        } else {
            pcl = getPointcloud(depthImage, depthCameraInfo, config.maximum_depth);
        }

        try {
            auto t = tfBuffer.lookupTransform(config.camera_frame, pcl->header.frame_id, depthImage->header.stamp);
            Eigen::Affine3d tt = tf2::transformToEigen(t);
            pcl::transformPointCloud(*pcl, *transformedPointCloud, tt);
            transformedPointCloud->header.frame_id = config.camera_frame;
            transformedPointCloud->header.stamp = pcl->header.stamp;
        } catch (const tf2::TransformException& e) {
            ROS_ERROR("%s", e.what());
            return false;
        }

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (transformedPointCloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            ROS_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }

        extractIndicesFilter.setIndices(inliers);
        extractIndicesFilter.setInputCloud(transformedPointCloud);
        extractIndicesFilter.filter(*planePointCloud);

        Eigen::Vector3f surface(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        surface /= coefficients->values[3];

        auto height = 1.0 / surface.norm();
        auto pitch = std::atan2(surface[0], surface[2]);
        auto roll = std::atan2(surface[1], surface[2]);
        auto yaw = 0.0;
        auto x = 0.0;
        auto y = 0.0;

        if(config.use_marker) {
            try {
                this->image = cv_bridge::toCvCopy(image);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
                return false;
            }
            auto& cvImage = this->image->image;
            imageCameraModel.fromCameraInfo(cameraInfo);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::detectMarkers(cvImage, dictionary, corners, ids);

            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(cvImage, corners, ids);
                cv::aruco::estimatePoseSingleMarkers(corners, static_cast<float>(config.aruco_size),
                                                     imageCameraModel.intrinsicMatrix(),
                                                     imageCameraModel.distortionCoeffs(), rvecs, tvecs);

                tf2::Quaternion tt;
                tf2::Stamped<tf2::Transform> imageToScrew;
                tf2::Stamped<tf2::Transform> markerToBaseLink;
                try {
                    geometry_msgs::TransformStamped t;
                    t = tfBuffer.lookupTransform(config.camera_frame, image->header.frame_id, ros::Time(0));
                    tf2::convert(t.transform.rotation, tt);
                    tf2::convert(t, imageToScrew);

                    t = tfBuffer.lookupTransform("marker", "base_link", ros::Time(0));
                    tf2::convert(t, markerToBaseLink);
                } catch (const tf2::TransformException& e) {
                    ROS_ERROR("%s", e.what());
                    return false;
                }
                for (int i = 0; i < ids.size(); i++) {
                    if (ids[i] != config.aruco_id) {
                        continue;
                    }

                    cv::aruco::drawAxis(cvImage, imageCameraModel.intrinsicMatrix(),
                                        imageCameraModel.distortionCoeffs(),
                                        rvecs[i], tvecs[i], 0.1);
                    cv::Mat1d cameraTransformRotation;
                    cv::Rodrigues(rvecs[i], cameraTransformRotation);


                    tf2::Matrix3x3 rotation
                            (cameraTransformRotation(0, 0), cameraTransformRotation(0, 1),
                             cameraTransformRotation(0, 2),
                             cameraTransformRotation(1, 0), cameraTransformRotation(1, 1),
                             cameraTransformRotation(1, 2),
                             cameraTransformRotation(2, 0), cameraTransformRotation(2, 1),
                             cameraTransformRotation(2, 2));

                    tf2::Transform arucoTransform(rotation, tf2::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
                    auto s = arucoTransform.inverse() * imageToScrew.inverse();
                    yaw = tf2::getYaw(s.getRotation());

                    x = s.getOrigin().x() - markerToBaseLink.getOrigin().x();
                    y = s.getOrigin().y() - markerToBaseLink.getOrigin().y();

                    break;
                }
            } else {
                ROS_ERROR("Marker not found!");
                return false;
            }
        }

        if (std::fabs(roll) > config.max_roll) {
            ROS_ERROR("Roll is bigger than configured threshold. Estimated %f but max threshold %f!", roll, config.max_roll);
            return false;
        }

        if (std::fabs(pitch) > config.max_pitch) {
            ROS_ERROR("Pitch is bigger than configured threshold. Estimated %f but max threshold %f!", pitch, config.max_pitch);
            return false;
        }

        if (std::fabs(yaw) > config.max_yaw) {
            ROS_ERROR("Yaw is bigger than configured threshold. Estimated %f but max threshold %f!", yaw, config.max_yaw);
            return false;
        }


        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "base_link";
        transform.header.stamp = ros::Time::now();
        transform.child_frame_id = config.camera_frame;
        transform.transform.translation.x = x + config.x_offset;
        transform.transform.translation.y = y + config.y_offset;
        transform.transform.translation.z = height + config.height_offset;
        tf2::Quaternion q;
        q.setRPY(roll + config.roll_offset, -pitch + config.pitch_offset, yaw + config.yaw_offset);
        geometry_msgs::Quaternion qq;
        tf2::convert(q, qq);
        transform.transform.rotation = qq;
        tfBroadcaster.sendTransform(transform);

        lastPoseEstimationTime = ros::Time::now();

        return true;
    }

    sensor_msgs::PointCloud2ConstPtr StereoCameraPoseEstimation::getPlaneCloud() {
        auto ret = boost::make_shared<sensor_msgs::PointCloud2>();
        pcl::toROSMsg(*planePointCloud, *ret);

        return ret;
    }

    sensor_msgs::ImagePtr StereoCameraPoseEstimation::getMarkerImage() {
        return image->toImageMsg();
    }
}
