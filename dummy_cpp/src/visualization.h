/*Comments*/
#ifndef Visualization_H
#define Visualization_H
#include <visualization_msgs/Marker.h>
#include <string>
/* Place to put all of my definitions etc. */
// Arguements are - publisher for path,pointsx,y, color as string
// color = "red","green", ""blue","rg","gb","br"
void visulize_path(ros::Publisher& p_path, std::vector<double> pts_x,std::vector<double> pts_y, std::string color){
    ros::Publisher ref_path_marker_pub;
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    if(color.compare("red")!=0)
      points.color.r = line_strip.color.r = 1.0f;
    else if(color.compare("green")!=0)
      points.color.g = line_strip.color.g = 1.0f;
    else if(color.compare( "blue")!=0)
      points.color.b = line_strip.color.b = 1.0f;
    else if(color.compare("rg")!=0 || color.compare("gr")!=0){
      points.color.r = line_strip.color.r = 1.0f;
      points.color.g = line_strip.color.b = 1.0f;
    }
    else if(color.compare(  "gb")!=0 || color.compare("bg")!=0){
      points.color.g = line_strip.color.r = 1.0f;
      points.color.b = line_strip.color.b = 1.0f;
    }
    else if(color.compare( "br")!=0 || color.compare( "rb")!=0){
      points.color.r = line_strip.color.r = 1.0f;
      points.color.b = line_strip.color.b = 1.0f;
    }

    points.color.a = line_strip.color.a = 1.0; //alpha , 1 makes marker visible

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < pts_x.size(); i++)
    {
      geometry_msgs::Point p;
      p.x = pts_x[i];
      p.y = pts_y[i];
      p.z = 0;
      points.points.push_back(p);
      line_strip.points.push_back(p);
    }
    p_path.publish(points);
    p_path.publish(line_strip);
}
#endif
