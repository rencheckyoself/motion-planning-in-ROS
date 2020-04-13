/// \file
/// \brief Node to draw the features of the real world map
///
/// PARAMETERS:
///     obstacles (std::string) the name of the odometer frame
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
/// PUBLISHES:
///     /visualization_marker_array (visualization_msgs::MarkerArray) markers

#include <vector>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "rigid2d/rigid2d.hpp"

/// \brief main function to create the real world map
int main(int argc, char** argv)
{

  ros::init(argc, argv, "draw_map");
  ros::NodeHandle n;

  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims;
  std::vector<double> map_y_lims;
  XmlRpc::XmlRpcValue obstacles;

  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("obstacles", obstacles);

  ROS_INFO_STREAM("MAP: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("MAP: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("MAP: Loaded Params");

  std::vector<std::vector<geometry_msgs::Point>> polygons;
  std::vector<geometry_msgs::Point> buf_poly, map_edge;
  geometry_msgs::Point buf_point;

  for(int i=0; i < obstacles.size(); i++) // loop through each obstacle
  {
    for(int j=0; j < obstacles[i].size(); j++) // loop through each point in the obstacle
    {
      buf_point.x = double(obstacles[i][j][0]);
      buf_point.y = double(obstacles[i][j][1]);
      buf_point.z = 0;
      buf_poly.push_back(buf_point);
    }

    buf_point.x = double(obstacles[i][0][0]);
    buf_point.y = double(obstacles[i][0][1]);
    buf_point.z = 0;
    buf_poly.push_back(buf_point);

    polygons.push_back(buf_poly);
    buf_poly.clear();
  }

  buf_point.x = map_x_lims.at(0);
  buf_point.y = map_y_lims.at(0);
  map_edge.push_back(buf_point);

  buf_point.x = map_x_lims.at(1);
  map_edge.push_back(buf_point);

  buf_point.y = map_y_lims.at(1);
  map_edge.push_back(buf_point);

  buf_point.x = map_x_lims.at(0);
  map_edge.push_back(buf_point);

  buf_point.y = map_y_lims.at(0);
  map_edge.push_back(buf_point);

  visualization_msgs::Marker marker;
  std::vector<visualization_msgs::Marker> markers;

  visualization_msgs::MarkerArray pub_marks;

  std::string ns = "world_map";
  int id = 0;
  float r = 33./255., g = 36./255., b = 61./255.;

  for(auto polygon : polygons)
  {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points = polygon;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.5;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    markers.push_back(marker);

    id++;
  }

  // Build Map Boarder Marker
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.points = map_edge;

  // marker.color.r = 0;
  // marker.color.b = 0;
  // marker.color.g = 0;

  markers.push_back(marker);

  pub_marks.markers = markers;

  pub_markers.publish(pub_marks);

  ros::spin();
}
