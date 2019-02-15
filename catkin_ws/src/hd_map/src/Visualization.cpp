#include <hd_map/HDMap.h>
#include <hd_map/Polyline2D.h>
#include <hd_map/Polyline3D.h>
#include <hd_map/Road.h>
#include <hd_map/Lane.h>
#include <hd_map/LaneGroup.h>
#include <hd_map/LaneGroupConnector.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/package.h>

#include "Visualization.h"
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hd_map_visualization");
    ros::NodeHandle nh("~");

    hd_map::Visualization visualization(nh);

    ros::spin();

    return 0;
}

hd_map::Visualization::Visualization(ros::NodeHandle &nh)
{
    map.loadMap();
    clickedPointSubscriber = nh.subscribe("/clicked_point", 1, &Visualization::onClickedPoint, this);

    roadsPublisher = nh.advertise<visualization_msgs::MarkerArray>("roads", 1, true);
    lanesPublisher = nh.advertise<visualization_msgs::MarkerArray>("lanes", 1, true);
    routePublisher = nh.advertise<visualization_msgs::MarkerArray>("route", 1);

    visualize();
}

void hd_map::Visualization::visualize() {
    visualization_msgs::MarkerArray roadMarkers;
    int id = 0;
    for (const auto &road : map.getRoads()) {
        visualization_msgs::Marker roadMarker;

        roadMarker.header.frame_id = "map";
        roadMarker.id = id++;

        roadMarker.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        roadMarker.scale.x = 1.0;
        // Line strip is blue
        roadMarker.color.r = 1.0;
        roadMarker.color.b = 1.0;
        roadMarker.color.g = 1.0;
        roadMarker.color.a = 1.0;

        // Create the vertices for the points and lines
        for (const auto &point : road->getReferenceTrack()->getPoints())
        {
            geometry_msgs::Point p;
            tf2::toMsg(point, p);
            roadMarker.points.push_back(p);
        }

        roadMarkers.markers.push_back(roadMarker);
    }

    visualization_msgs::MarkerArray laneMarkers;
    for (const auto &laneGroup : map.getLaneGroups()) {
        int laneIndex = 0;
        for (const auto &lane : laneGroup->getLanes()) {
            visualization_msgs::Marker startLaneMappingMarker;
            {
                startLaneMappingMarker.header.frame_id = "map";
                startLaneMappingMarker.id = id++;
                startLaneMappingMarker.ns = "Lane reference track";

                startLaneMappingMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                startLaneMappingMarker.scale.z = 0.01;

                startLaneMappingMarker.color.r = 1.0;
                startLaneMappingMarker.color.b = 0.0;
                startLaneMappingMarker.color.g = 1.0;
                startLaneMappingMarker.color.a = 1.0;
                //startLaneMappingMarker.text = std::to_string(std::get<0>(laneGroup->getLaneMapping().at(laneIndex)));


                auto point = lane->getReferenceTrack()->getPoints().front();
                // Create the vertices for the points and lines
                if (lane->getRelativeDirection() == RelativeDirection::BACKWARD) {
                    point = lane->getReferenceTrack()->getPoints().back();
                }

                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                p.y += 1;
                startLaneMappingMarker.pose.position = p;
                laneMarkers.markers.push_back(startLaneMappingMarker);
            }

            {
                visualization_msgs::Marker endLaneMappingMarker;

                endLaneMappingMarker.header.frame_id = "map";
                endLaneMappingMarker.id = id++;
                endLaneMappingMarker.ns = "Lane reference track";

                endLaneMappingMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                endLaneMappingMarker.scale.z = 0.01;

                endLaneMappingMarker.color.r = 1.0;
                endLaneMappingMarker.color.b = 0.0;
                endLaneMappingMarker.color.g = 0.0;
                endLaneMappingMarker.color.a = 1.0;
                //endLaneMappingMarker.text = std::to_string(std::get<1>(laneGroup->getLaneMapping().at(laneIndex++)));


                auto point = lane->getReferenceTrack()->getPoints().back();
                // Create the vertices for the points and lines
                if (lane->getRelativeDirection() == RelativeDirection::BACKWARD) {
                    point = lane->getReferenceTrack()->getPoints().front();
                }

                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                p.y -= 1;
                endLaneMappingMarker.pose.position = p;
                laneMarkers.markers.push_back(endLaneMappingMarker);
            }

            visualization_msgs::Marker laneMarker;

            laneMarker.header.frame_id = "map";
            laneMarker.id = id++;
            laneMarker.ns = "Lane reference track";

            laneMarker.type = visualization_msgs::Marker::LINE_STRIP;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            laneMarker.scale.x = 0.01;

            laneMarker.color.r = 1.0;
            laneMarker.color.b = 0.0;
            laneMarker.color.g = 1.0;
            laneMarker.color.a = 1.0;


            // Create the vertices for the points and lines
            for (const auto &point : lane->getReferenceTrack()->getPoints()) {
                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                laneMarker.points.push_back(p);
            }
            for (int i = 0; i < lane->getReferenceTrack()->getPoints().size(); i++) {
                std_msgs::ColorRGBA laneColor;
                laneColor.r = (float) i / (float)lane->getReferenceTrack()->getPoints().size();
                laneColor.b = laneGroup->isJunction() ? 1.0 : 0.0;
                laneColor.g = 0;
                laneColor.a = 1.0;
                laneMarker.colors.push_back(laneColor);
            }

            laneMarkers.markers.push_back(laneMarker);


            // left boundary
            visualization_msgs::Marker leftBoundaryMarker;
            leftBoundaryMarker.header.frame_id = "map";
            leftBoundaryMarker.id = id++;
            leftBoundaryMarker.ns = "Left lane boundary";

            leftBoundaryMarker.type = visualization_msgs::Marker::LINE_STRIP;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            leftBoundaryMarker.scale.x = 0.01;
            // Line strip is blue
            leftBoundaryMarker.color.r = 1.0;
            leftBoundaryMarker.color.b = 0.0;
            leftBoundaryMarker.color.g = 1.0;
            leftBoundaryMarker.color.a = 1.0;

            // Create the vertices for the points and lines
            for (const auto &point : lane->getLeftBoundary()->getPoints()) {
                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                leftBoundaryMarker.points.push_back(p);
            }

            for (int i = 0; i < lane->getLeftBoundary()->getPoints().size(); i++) {
                std_msgs::ColorRGBA laneColor;
                laneColor.r = (float) i / (float) lane->getLeftBoundary()->getPoints().size();
                laneColor.b = 1.0;
                laneColor.g = 1.0;
                laneColor.a = 1.0;
                leftBoundaryMarker.colors.push_back(laneColor);
            }


            laneMarkers.markers.push_back(leftBoundaryMarker);


            // right boundary
            visualization_msgs::Marker rightBoundaryMarker;

            rightBoundaryMarker.header.frame_id = "map";
            rightBoundaryMarker.id = id++;
            rightBoundaryMarker.ns = "Right lane boundary";

            rightBoundaryMarker.type = visualization_msgs::Marker::LINE_STRIP;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            rightBoundaryMarker.scale.x = 0.01;
            // Line strip is blue
            rightBoundaryMarker.color.r = 1.0;
            rightBoundaryMarker.color.b = 0.0;
            rightBoundaryMarker.color.g = 1.0;
            rightBoundaryMarker.color.a = 1.0;

            // Create the vertices for the points and lines
            for (const auto &point : lane->getRightBoundary()->getPoints()) {
                geometry_msgs::Point p;
                tf2::toMsg(point, p);

                rightBoundaryMarker.points.push_back(p);
            }

            for (int i = 0; i < lane->getRightBoundary()->getPoints().size(); i++) {
                std_msgs::ColorRGBA laneColor;
                laneColor.r = (float) i / (float)lane->getRightBoundary()->getPoints().size();
                laneColor.b = 1.0;
                laneColor.g = 1.0;
                laneColor.a = 1.0;
                rightBoundaryMarker.colors.push_back(laneColor);
            }

            laneMarkers.markers.push_back(rightBoundaryMarker);
        }

        // Lane group connectors
        if (laneGroup->getStartConnector()) {
            visualization_msgs::Marker startLaneConnectorMarker;

            startLaneConnectorMarker.header.frame_id = "map";
            startLaneConnectorMarker.id = id++;
            startLaneConnectorMarker.ns = "Start lane connector";

            startLaneConnectorMarker.type = visualization_msgs::Marker::LINE_STRIP;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            startLaneConnectorMarker.scale.x = 0.01;
            // Line strip is blue
            startLaneConnectorMarker.color.r = 1.0;
            startLaneConnectorMarker.color.b = 0.0;
            startLaneConnectorMarker.color.g = 1.0;
            startLaneConnectorMarker.color.a = 1.0;

            for (auto point : laneGroup->getStartConnector()->getBoundaryGeometry()->getPoints()) {
                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                startLaneConnectorMarker.points.push_back(p);
            }
            laneMarkers.markers.push_back(startLaneConnectorMarker);

            visualization_msgs::Marker startLaneConnectorTextMarker;
            startLaneConnectorTextMarker.header.frame_id = "map";
            startLaneConnectorTextMarker.id = id++;
            startLaneConnectorTextMarker.ns = "Start lane connector";

            startLaneConnectorTextMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            startLaneConnectorTextMarker.scale.z = 1;
            // Line strip is blue
            startLaneConnectorTextMarker.color.r = 1.0;
            startLaneConnectorTextMarker.color.b = 0.0;
            startLaneConnectorTextMarker.color.g = 1.0;
            startLaneConnectorTextMarker.color.a = 1.0;
            startLaneConnectorTextMarker.text = "Start: " + std::to_string(laneGroup->getStartConnector()->getId()) + " Lane group: " + std::to_string(laneGroup->getId());
            auto first = laneGroup->getStartConnector()->getBoundaryGeometry()->getPoints().front();
            geometry_msgs::Point p;
            tf2::toMsg(first, p);
            p.y += 1;
            startLaneConnectorTextMarker.points.push_back(p);

            startLaneConnectorTextMarker.pose.position = p;
            laneMarkers.markers.push_back(startLaneConnectorTextMarker);
        }


        if (laneGroup->getEndConnector()) {
            visualization_msgs::Marker endLaneConnectorMarker;

            endLaneConnectorMarker.header.frame_id = "map";
            endLaneConnectorMarker.id = id++;
            endLaneConnectorMarker.ns = "End lane connector";

            endLaneConnectorMarker.type = visualization_msgs::Marker::LINE_STRIP;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            endLaneConnectorMarker.scale.x = 0.2;
            // Line strip is blue
            endLaneConnectorMarker.color.r = 1.0;
            endLaneConnectorMarker.color.b = 0.0;
            endLaneConnectorMarker.color.g = 0.0;
            endLaneConnectorMarker.color.a = 1.0;

            for (auto point : laneGroup->getEndConnector()->getBoundaryGeometry()->getPoints()) {
                geometry_msgs::Point p;
                tf2::toMsg(point, p);
                endLaneConnectorMarker.points.push_back(p);
            }
            laneMarkers.markers.push_back(endLaneConnectorMarker);

            visualization_msgs::Marker endLaneConnectorTextMarker;
            endLaneConnectorTextMarker.header.frame_id = "map";
            endLaneConnectorTextMarker.id = id++;
            endLaneConnectorTextMarker.ns = "End lane connector";

            endLaneConnectorTextMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            endLaneConnectorTextMarker.scale.z = 1;
            // Line strip is blue
            endLaneConnectorTextMarker.color.r = 1.0;
            endLaneConnectorTextMarker.color.b = 0.0;
            endLaneConnectorTextMarker.color.g = 0.0;
            endLaneConnectorTextMarker.color.a = 1.0;
            endLaneConnectorTextMarker.text = "End: " + std::to_string(laneGroup->getEndConnector()->getId()) + " Lane group: " + std::to_string(laneGroup->getId());
            auto first = laneGroup->getEndConnector()->getBoundaryGeometry()->getPoints().back();
            geometry_msgs::Point p;
            tf2::toMsg(first, p);
            p.y -= 1;
            endLaneConnectorMarker.points.push_back(p);

            endLaneConnectorTextMarker.pose.position = p;
            laneMarkers.markers.push_back(endLaneConnectorTextMarker);
        }

    }

    roadsPublisher.publish(roadMarkers);
    lanesPublisher.publish(laneMarkers);
}

void hd_map::Visualization::onClickedPoint(const geometry_msgs::PointStampedConstPtr& point) {
    if (!start) {
        start = point;
        return;
    }

    if (!end) {
        end = point;
    }

    visualization_msgs::MarkerArray deleteMarkers;
    visualization_msgs::Marker deleteMarker;
    deleteMarker.header.frame_id = "map";
    deleteMarker.ns = "Routing";
    deleteMarker.type = visualization_msgs::Marker::LINE_STRIP;
    deleteMarker.action = visualization_msgs::Marker::DELETEALL;
    deleteMarkers.markers.push_back(deleteMarker);
    routePublisher.publish(deleteMarkers);

    int id = 0;
    tf2::Vector3 s(start->point.x, start->point.y, start->point.z);
    tf2::Vector3 e(end->point.x, end->point.y, end->point.z);
    const auto& route = map.findRoute(s, e);
    visualization_msgs::MarkerArray routeMarkers;
    for (const auto& lane : route) {
        visualization_msgs::Marker laneMarker;

        laneMarker.header.frame_id = "map";
        laneMarker.id = id++;
        laneMarker.ns = "Routing";

        laneMarker.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        laneMarker.scale.x = 0.02;

        laneMarker.color.r = 1.0;
        laneMarker.color.b = 0.0;
        laneMarker.color.g = 1.0;
        laneMarker.color.a = 1.0;


        // Create the vertices for the points and lines
        for (const auto &point : lane->getReferenceTrack()->getPoints()) {
            geometry_msgs::Point p;
            tf2::toMsg(point, p);
            laneMarker.points.push_back(p);
        }
        for (int i = 0; i < lane->getReferenceTrack()->getPoints().size(); i++) {
            std_msgs::ColorRGBA laneColor;
            laneColor.r = (float) i / (float)lane->getReferenceTrack()->getPoints().size();
            laneColor.b = lane->getLaneGroup()->isJunction() ? 1.0 : 0.0;
            laneColor.g = 0;
            laneColor.a = 1.0;
            laneMarker.colors.push_back(laneColor);
        }

        routeMarkers.markers.push_back(laneMarker);
    }
    routePublisher.publish(routeMarkers);

    start = nullptr;
    end = nullptr;
}
