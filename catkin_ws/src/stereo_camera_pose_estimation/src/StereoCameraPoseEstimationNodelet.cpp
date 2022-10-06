#include "stereo_camera_pose_estimation/StereoCameraPoseEstimation.h"

namespace stereo_camera_pose_estimation {
    StereoCameraPoseEstimationNodelet::StereoCameraPoseEstimationNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("stero_camera_position_estimation", opts),
                                                                                                            tfBroadcaster(this),
                                                                                                            tfBuffer(get_clock()),
                                                                                                            tfListener(tfBuffer) {
        config.maximum_depth = declare_parameter<double>("maximum_depth", 2.0);
        config.aruco_id = declare_parameter<int>("aruco_id", 0);
        config.aruco_size = declare_parameter<double>("aruco_size", 0.02);
        config.max_yaw = declare_parameter<double>("max_yaw", 0.785);
        config.max_pitch = declare_parameter<double>("max_pitch", 0.785);
        config.max_roll = declare_parameter<double>("max_roll", 0.785);
        config.idle_time = declare_parameter<double>("idle_time", 10);
        config.x_offset = declare_parameter<double>("x_offset", 0.0);
        config.y_offset = declare_parameter<double>("y_offset", 0.0);
        config.height_offset = declare_parameter<double>("height_offset", 0.0);
        config.yaw_offset = declare_parameter<double>("yaw_offset", 0.0);
        config.pitch_offset = declare_parameter<double>("pitch_offset", 0.0);
        config.roll_offset = declare_parameter<double>("roll_offset", 0.0);
        config.use_marker = declare_parameter<bool>("use_marker", true);
        config.camera_frame = declare_parameter<std::string>("camera_frame", "camera_bottom_screw_frame");
        config.base_link_frame = declare_parameter<std::string>("base_link_frame", "base_link");
        config.marker_frame = declare_parameter<std::string>("marker_frame", "marker");
        config.debug = declare_parameter<bool>("debug", true);

        coefficients = std::make_shared<pcl::ModelCoefficients>();
        planePointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        transformedPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

        planeCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("plane_pcl", 10);

        markerImagePublisher = create_publisher<sensor_msgs::msg::Image>("marker", 1);

        auto qos = rclcpp::QoS(2).get_rmw_qos_profile();
        infraImageSubscriber.subscribe(this, "camera/color/image_rect_color", "raw", qos);
        depthImageSubscriber.subscribe(this, "camera/depth/image_rect_raw", "raw", qos);
        infraCameraInfoSubscriber.subscribe(this, "camera/color/camera_info", qos);
        depthCameraInfoSubscriber.subscribe(this, "camera/depth/camera_info", qos);

        sync = std::make_shared<SynchronizerDepthImage>(SyncPolicyDepthImage(20));
        sync->connectInput(infraImageSubscriber, infraCameraInfoSubscriber, depthImageSubscriber,
                           depthCameraInfoSubscriber);
        sync->registerCallback(std::bind(&StereoCameraPoseEstimationNodelet::onImage, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    void StereoCameraPoseEstimationNodelet::onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg,
                                                    const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
                                                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depth_camera_info) {

        if (processImage(msg, info_msg, depth_image, depth_camera_info) && config.debug) {
            planeCloudPublisher->publish(*getPlaneCloud());
            if(config.use_marker) {
                markerImagePublisher->publish(*getMarkerImage());
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    StereoCameraPoseEstimationNodelet::getPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo,
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

        auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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
        pointcloud->header.frame_id = depthImage->header.frame_id;
        pointcloud->header.stamp = rclcpp::Time(depthImage->header.stamp).nanoseconds() / 1000ull;

        return pointcloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    StereoCameraPoseEstimationNodelet::getPointcloudFloat(const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo,
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

        auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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
        pointcloud->header.frame_id = depthImage->header.frame_id;
        pointcloud->header.stamp = rclcpp::Time(depthImage->header.stamp).nanoseconds() / 1000ull;

        return pointcloud;
    }

    bool StereoCameraPoseEstimationNodelet::processImage(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cameraInfo,
                                                         const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                         const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo) {

        auto durationSinceLastCalibration = now() - lastPoseEstimationTime;
        if (durationSinceLastCalibration.seconds() < config.idle_time) {
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
            RCLCPP_ERROR(get_logger(), "%s", e.what());
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
            RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given dataset.");
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
                RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
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
                    geometry_msgs::msg::TransformStamped t;
                    t = tfBuffer.lookupTransform(config.camera_frame, image->header.frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME));
                    tf2::convert(t.transform.rotation, tt);
                    tf2::convert(t, imageToScrew);

                    t = tfBuffer.lookupTransform(config.marker_frame, config.base_link_frame, rclcpp::Time(0, 0, RCL_ROS_TIME));
                    tf2::convert(t, markerToBaseLink);
                } catch (const tf2::TransformException& e) {
                    RCLCPP_ERROR(get_logger(), "%s", e.what());
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
                RCLCPP_ERROR(get_logger(), "Marker not found!");
                return false;
            }
        }

        if (std::fabs(roll) > config.max_roll) {
            RCLCPP_ERROR(get_logger(), "Roll is bigger than configured threshold. Estimated %f but max threshold %f!", roll, config.max_roll);
            return false;
        }

        if (std::fabs(pitch) > config.max_pitch) {
            RCLCPP_ERROR(get_logger(), "Pitch is bigger than configured threshold. Estimated %f but max threshold %f!", pitch, config.max_pitch);
            return false;
        }

        if (std::fabs(yaw) > config.max_yaw) {
            RCLCPP_ERROR(get_logger(), "Yaw is bigger than configured threshold. Estimated %f but max threshold %f!", yaw, config.max_yaw);
            return false;
        }


        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = config.base_link_frame;
        transform.header.stamp = now();
        transform.child_frame_id = config.camera_frame;
        transform.transform.translation.x = x + config.x_offset;
        transform.transform.translation.y = y + config.y_offset;
        transform.transform.translation.z = height + config.height_offset;
        tf2::Quaternion q;
        q.setRPY(roll + config.roll_offset, -pitch + config.pitch_offset, yaw + config.yaw_offset);
        geometry_msgs::msg::Quaternion qq;
        tf2::convert(q, qq);
        transform.transform.rotation = qq;
        tfBroadcaster.sendTransform(transform);

        lastPoseEstimationTime = now();


        return true;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr StereoCameraPoseEstimationNodelet::getPlaneCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*planePointCloud, *ret);

        return ret;
    }

    sensor_msgs::msg::Image::SharedPtr StereoCameraPoseEstimationNodelet::getMarkerImage() {
        return image->toImageMsg();
    }
}