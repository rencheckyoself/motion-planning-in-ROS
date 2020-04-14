/// \file
/// \brief Node to draw the features of the real world map
///
/// PARAMETERS:
///     obstacles (std::vector<std::vector<std::vector<double>) a vector of polygons represented by a vector of x,y coords for the verticies
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
///     r (std::vector<int>) color values
///     g (std::vector<int>) color values
///     b (std::vector<int>) color values
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
  std::vector<double> r, g, b;
  XmlRpc::XmlRpcValue obstacles;

  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("r", r);
  n.getParam("g", g);
  n.getParam("b", b);
  n.getParam("obstacles", obstacles);

  for(unsigned int i = 0; i < r.size(); i++)
  {
    r.at(i) /= 255;
    g.at(i) /= 255;
    b.at(i) /= 255;
  }

  ROS_INFO_STREAM("MAP: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("MAP: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("MAP: Loaded Params");

  std::vector<std::vector<geometry_msgs::Point>> polygons;
  std::vector<geometry_msgs::Point> buf_poly, map_edge;
  geometry_msgs::Point buf_point;

  for(int i=0; i < obstacles.size(); i++) // loop through each obstacle
  {
    buf_point.x = double(obstacles[i][0][0]);
    buf_point.y = double(obstacles[i][0][1]);
    buf_point.z = 0;
    buf_poly.push_back(buf_point);

    for(int j=1; j < obstacles[i].size(); j++) // loop through each point in the obstacle
    {
      buf_point.x = double(obstacles[i][j][0]);
      buf_point.y = double(obstacles[i][j][1]);
      buf_point.z = 0;
      buf_poly.push_back(buf_point);
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
  map_edge.push_back(buf_point);

  buf_point.y = map_y_lims.at(1);
  map_edge.push_back(buf_point);
  map_edge.push_back(buf_point);

  buf_point.x = map_x_lims.at(0);
  map_edge.push_back(buf_point);
  map_edge.push_back(buf_point);

  buf_point.y = map_y_lims.at(0);
  map_edge.push_back(buf_point);

  visualization_msgs::Marker marker;
  std::vector<visualization_msgs::Marker> markers;

  visualization_msgs::MarkerArray pub_marks;

  std::string ns = "world_map";
  int id = 0;

  for(auto polygon : polygons)
  {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points = polygon;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.25;

    marker.color.r = r.at(3);
    marker.color.g = g.at(3);
    marker.color.b = b.at(3);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    markers.push_back(marker);

    id++;
  }

  // Build Map Boarder Marker
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.points = map_edge;

  markers.push_back(marker);

  pub_marks.markers = markers;

  pub_markers.publish(pub_marks);

  ros::spin();
}
