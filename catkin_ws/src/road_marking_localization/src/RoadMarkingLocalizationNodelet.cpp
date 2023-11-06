#include "road_marking_localization/RoadMarkingLocalization.h"

namespace road_marking_localization {
    RoadMarkingLocalizationNodelet::RoadMarkingLocalizationNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("road_marking_localization", opts), tfBuffer(get_clock()), tfListener(tfBuffer) {
        config.blur_kernel_size = declare_parameter<int>("blur_kernel_size", 1);
        config.crop_top_pixels = declare_parameter<int>("crop_top_pixels", 50);
        config.x_box = declare_parameter<double>("x_box", 3.0);
        config.y_box = declare_parameter<double>("y_box", 3.0);
        config.minimum_z = declare_parameter<double>("minimum_z", -0.02);
        config.maximum_z = declare_parameter<double>("maximum_z", 0.02);
        config.threshold = declare_parameter<int>("threshold", 200);
        config.icp_max_iterations = declare_parameter<int>("icp_max_iterations", 5);
        config.icp_RANSAC_outlier_rejection_threshold = declare_parameter<double>("icp_RANSAC_outlier_rejection_threshold", 0.0);
        config.icp_RANSAC_iterations = declare_parameter<int>("icp_RANSAC_iterations", 0);
        config.icp_max_correspondence_distance = declare_parameter<double>("icp_max_correspondence_distance", 0.10);
        config.icp_sample_size = declare_parameter<int>("icp_sample_size", 750);
        config.minimum_points = declare_parameter<int>("minimum_points", 500);
        config.maximum_x_correction = declare_parameter<double>("maximum_x_correction", 0.3);
        config.maximum_y_correction = declare_parameter<double>("maximum_y_correction", 0.3);
        config.maximum_yaw_correction = declare_parameter<double>("maximum_yaw_correction", 0.5);
        config.debug = declare_parameter<bool>("debug", true);
        std::string base_link_frame = declare_parameter<std::string>("base_link_frame", "base_link");
        std::string map_frame = declare_parameter<std::string>("map_frame", "map");
        config.transformation_estimation = declare_parameter<int>("transformation_estimation", 0);

        croppedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        randomSampledCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        alignedPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        transformedCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        std::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
                (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

        // Create a TransformationEstimationLM object, and set the warp to it
        auto lmTE = new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>();
        lmTE->setWarpFunction(warp_fcn);

        transformationEstimation.reset(lmTE);

        // Pass the TransformationEstimation object to the ICP algorithm
        iterativeClosestPoint.setTransformationEstimation(transformationEstimation);

        auto pr = std::make_shared<pcl::CustomPointRepresentation<pcl::PointXYZ>>(2);
        iterativeClosestPoint.setPointRepresentation(pr);

        randomSampleFilter.setSeed(static_cast<unsigned int>(rand()));

        transformationMatrix.setIdentity();
        setConfig();

        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("corrected_odom", 10);
        thresholdedImagePublisher = create_publisher<sensor_msgs::msg::Image>("threshold", 1);
        rawPclPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("raw_pcl", 1);
        croppedPclPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("cropped_pcl", 1);
        randomSampledPclPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("random_sampled_pcl", 1);
        alignedPclPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("aligned_pcl", 1);
        auto qos = rclcpp::QoS(1).transient_local();
        mapCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("map_pcl", qos);
        robotLocalizationSetPose = create_client<robot_localization::srv::SetPose>("/sensors/set_pose");

        mapSubscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos, std::bind(&RoadMarkingLocalizationNodelet::onMap, this, std::placeholders::_1));
        positionEstimateSubscriber = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&RoadMarkingLocalizationNodelet::onEstimatedPosition, this, std::placeholders::_1));

        auto qos2 = rclcpp::QoS(2).get_rmw_qos_profile();
        infraImageSubscriber.subscribe(this, "camera/infra1/image_rect_raw", "raw", qos2);
        depthImageSubscriber.subscribe(this, "camera/depth/image_rect_raw", "raw", qos2);
        infraCameraInfoSubscriber.subscribe(this, "camera/infra1/camera_info", qos2);
        depthCameraInfoSubscriber.subscribe(this, "camera/depth/camera_info", qos2);

        sync = std::make_shared<SynchronizerDepthImage>(SyncPolicyDepthImage(2));
        sync->connectInput(infraImageSubscriber, infraCameraInfoSubscriber, depthImageSubscriber,
                           depthCameraInfoSubscriber);
        sync->registerCallback(std::bind(&RoadMarkingLocalizationNodelet::onImage, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

        cb = add_on_set_parameters_callback(std::bind(&RoadMarkingLocalizationNodelet::onConfig, this, std::placeholders::_1));
    }

    void RoadMarkingLocalizationNodelet::onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                                                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg,
                                                 const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
                                                 const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depth_camera_info) {

        if (processImage(msg, info_msg, depth_image, depth_camera_info)) {
            odometryPublisher->publish(getCorrectedPosition());

            if(config.debug) {
                rawPclPublisher->publish(*getRawPointCloud());
                croppedPclPublisher->publish(*getCroppedPointCloud());
                randomSampledPclPublisher->publish(*getRandomSampledPointCloud());
                alignedPclPublisher->publish(*getAlignedPointCloud());
                thresholdedImagePublisher->publish(*getThresholdedImage());
            }
        }
    }

    void RoadMarkingLocalizationNodelet::onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
        setMap(msg);
        mapCloudPublisher->publish(*getMapPointCloud());
    }

    void RoadMarkingLocalizationNodelet::onEstimatedPosition(const geometry_msgs::msg::PoseWithCovarianceStamped msg) {
        setPosition(msg);
        odometryPublisher->publish(getCorrectedPosition());
        auto srv = std::make_shared<robot_localization::srv::SetPose::Request>();
        srv->pose = msg;
        srv->pose.header.stamp = now();
        robotLocalizationSetPose->async_send_request(srv);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    RoadMarkingLocalizationNodelet::getPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo,
                                                  const cv::Mat &mask) {
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
        pointcloud->header.stamp = rclcpp::Time(depthImage->header.stamp).nanoseconds() / 1000ull;
        pointcloud->header.frame_id = depthImage->header.frame_id;

        return pointcloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr
    RoadMarkingLocalizationNodelet::getPointcloudFloat(const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo,
                                                       const cv::Mat &mask) {
        model.fromCameraInfo(depthCameraInfo);

        auto center_x = static_cast<float>(model.cx());
        auto center_y = static_cast<float>(model.cy());

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = 1.0;
        auto constant_x = static_cast<float>(unit_scaling / model.fx());
        auto constant_y = static_cast<float>(unit_scaling / model.fy());

        auto depth_row = reinterpret_cast<const float*>(&depthImage->data[0]);
        int row_step = depthImage->step / sizeof(float);

        auto pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pointcloud->reserve(100000);
        for (int v = 0; v < (int) depthImage->height; ++v, depth_row += row_step) {
            for (int u = 0; u < (int) depthImage->width; ++u) {
                auto depth = depth_row[u];

                // Skip missing points
                if (!std::isfinite(depth) || mask.at<char>(v, u) == 0) {
                    continue;
                }

                pointcloud->push_back(pcl::PointXYZ(
                        (u - center_x) * depth * constant_x, (v - center_y) * depth * constant_y, depth));
            }
        }


        pointcloud->height = 1;
        pointcloud->width = static_cast<uint32_t>(pointcloud->points.size());
        pointcloud->is_dense = true;
        pointcloud->header.stamp = rclcpp::Time(depthImage->header.stamp).nanoseconds() / 1000ull;
        pointcloud->header.frame_id = depthImage->header.frame_id;

        return pointcloud;
    }

    bool RoadMarkingLocalizationNodelet::processImage(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cameraInfo,
                                                      const sensor_msgs::msg::Image::ConstSharedPtr &depthImage,
                                                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &depthCameraInfo) {

        try {
            cv = cv_bridge::toCvCopy(image);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
            return false;
        }
        auto& cvImage = cv->image;

        cv::Mat cropRoi = cvImage(cv::Rect(0, 0, image->width, config.crop_top_pixels));
        cropRoi.setTo(cv::Scalar(0));
        cv::blur(cvImage, cvImage, cv::Size(config.blur_kernel_size, config.blur_kernel_size));
        cv::threshold(cvImage, cvImage, config.threshold, 255, CV_THRESH_BINARY);

        pcl::PointCloud<pcl::PointXYZ>::Ptr roadMarkerCloud;
        if (depthImage->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            roadMarkerCloud = getPointcloudFloat(depthImage, depthCameraInfo, cvImage);
        } else {
            roadMarkerCloud = getPointcloud(depthImage, depthCameraInfo, cvImage);
        }

        Eigen::Affine3d t;
        Eigen::Affine3d baseLinkTransform;
        try {
            tfBuffer.canTransform(config.map_frame, roadMarkerCloud->header.frame_id, depthImage->header.stamp, rclcpp::Duration::from_seconds(0.03));
            t = tf2::transformToEigen(tfBuffer.lookupTransform(config.map_frame, roadMarkerCloud->header.frame_id, depthImage->header.stamp));
            baseLinkTransform = tf2::transformToEigen(tfBuffer.lookupTransform(config.map_frame, config.base_link_frame, depthImage->header.stamp));
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
            return false;
        }

        pcl::transformPointCloud(*roadMarkerCloud, *transformedCloud, t);
        transformedCloud->header.frame_id = config.map_frame;

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
                auto clk = *get_clock();
                RCLCPP_WARN_THROTTLE(get_logger(), clk, 5000,"Not enough points for correction");
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
                RCLCPP_WARN_STREAM(get_logger(), "Too large transformation, not doing anything" << transformationMatrix);
                transformationMatrix = transformationMatrix.setIdentity();
                return false;
            }

            auto currentPos = Eigen::Vector4f(
                    static_cast<const float&>(baseLinkTransform.translation().x()),
                    static_cast<const float&>(baseLinkTransform.translation().y()),
                    static_cast<const float&>(baseLinkTransform.translation().z()), 1);
            auto pos = transformationMatrix * currentPos;
            correctedPosition.header.stamp = depthImage->header.stamp;
            correctedPosition.header.frame_id = config.map_frame;
            correctedPosition.child_frame_id = config.base_link_frame;
            correctedPosition.pose.pose.position.x = pos[0];
            correctedPosition.pose.pose.position.y = pos[1];
            correctedPosition.pose.pose.position.z = pos[2];
            correctedPosition.pose.covariance = { 0.001, 0, 0, 0, 0, 0,
                                                  0, 0.001, 0, 0, 0, 0,
                                                  0, 0, 0.001, 0, 0, 0,
                                                  0, 0, 0, 0.001, 0, 0,
                                                  0, 0, 0, 0, 0.001, 0,
                                                  0, 0, 0, 0, 0, 0.001
            };
            tf2::Quaternion q;

            q.setRPY(0, 0, baseLinkTransform.rotation().eulerAngles(0, 1, 2)[2] + yaw);
            tf2::convert(q, correctedPosition.pose.pose.orientation);
        } else {
            return false;
        }

        return true;
    }

    void RoadMarkingLocalizationNodelet::setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
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
        tmpMap->header.stamp = rclcpp::Time(msg->header.stamp).nanoseconds() / 1000ull;
        tmpMap->header.frame_id = msg->header.frame_id;

        mapPointCloud = tmpMap;

        iterativeClosestPoint.setInputTarget(mapPointCloud);
    }

    void RoadMarkingLocalizationNodelet::setPosition(const geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
        geometry_msgs::msg::Pose tfPose = pose.pose.pose;

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tfBuffer.lookupTransform(config.map_frame, pose.header.frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME));
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
        tf2::doTransform(tfPose, tfPose, t);

        correctedPosition.header.stamp = pose.header.stamp;
        correctedPosition.header.frame_id = config.map_frame;
        correctedPosition.child_frame_id = config.base_link_frame;

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

    sensor_msgs::msg::Image::ConstSharedPtr RoadMarkingLocalizationNodelet::getThresholdedImage() {
        return cv->toImageMsg();
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr RoadMarkingLocalizationNodelet::getRawPointCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*transformedCloud, *ret);

        return ret;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr RoadMarkingLocalizationNodelet::getCroppedPointCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*croppedCloud, *ret);

        return ret;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr RoadMarkingLocalizationNodelet::getRandomSampledPointCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*randomSampledCloud, *ret);

        return ret;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr RoadMarkingLocalizationNodelet::getAlignedPointCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*alignedPointCloud, *ret);

        return ret;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr RoadMarkingLocalizationNodelet::getMapPointCloud() {
        auto ret = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*mapPointCloud, *ret);

        return ret;
    }

    void RoadMarkingLocalizationNodelet::setConfig() {
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
            case 0: {
                std::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn
                        (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

                auto lmTE = new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>();
                lmTE->setWarpFunction(warp_fcn);

                transformationEstimation.reset(lmTE);
                RCLCPP_INFO(get_logger(), "Tranformation estimation using Levenberg Marquardt");
                break;
            }
            case 1: {
                auto teCoM = new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teCoM);
                RCLCPP_INFO(get_logger(), "Tranformation estimation using center of mass");
                break;
            }
            case 2: {
                auto teSVD = new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teSVD);
                RCLCPP_INFO(get_logger(), "Tranformation estimation using SVD");
                break;
            }

            case 3: {
                auto teDQ = new pcl::registration::TransformationEstimationDualQuaternion<pcl::PointXYZ, pcl::PointXYZ>();
                transformationEstimation.reset(teDQ);
                RCLCPP_INFO(get_logger(), "Tranformation estimation using dual quaternion");
                break;
            }

                iterativeClosestPoint.setTransformationEstimation(transformationEstimation);
        }
    }

rcl_interfaces::msg::SetParametersResult
RoadMarkingLocalizationNodelet::onConfig(const std::vector<rclcpp::Parameter> &params) {

    for (auto&& p : params) {
        const auto& name = p.get_name();

        if (name == "blur_kernel_size") config.blur_kernel_size = p.as_int();
        if (name == "crop_top_pixels") config.crop_top_pixels = p.as_int();
        if (name == "x_box") config.x_box = p.as_double();
        if (name == "y_box") config.y_box = p.as_double();
        if (name == "minimum_z") config.minimum_z = p.as_double();
        if (name == "maximum_z") config.maximum_z = p.as_double();
        if (name == "threshold") config.threshold = p.as_int();
        if (name == "icp_max_iterations") config.icp_max_iterations = p.as_int();
        if (name == "icp_RANSAC_outlier_rejection_threshold") config.icp_RANSAC_outlier_rejection_threshold = p.as_double();
        if (name == "icp_RANSAC_iterations") config.icp_RANSAC_iterations = p.as_double();
        if (name == "icp_max_correspondence_distance") config.icp_max_correspondence_distance = p.as_double();
        if (name == "icp_sample_size") config.icp_sample_size = p.as_int();
        if (name == "minimum_points") config.minimum_points = p.as_int();
        if (name == "maximum_x_correction") config.maximum_x_correction = p.as_double();
        if (name == "maximum_y_correction") config.maximum_y_correction = p.as_double();
        if (name == "maximum_yaw_correction") config.maximum_yaw_correction = p.as_double();
        if (name == "debug") config.debug = p.as_bool();
        if (name == "transformation_estimation") config.transformation_estimation = p.as_int();
    }

    setConfig();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
}

}
