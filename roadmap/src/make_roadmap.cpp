/// \file
/// \brief Node to create and draw a probabilistic road map
///
/// PARAMETERS:
///     obstacles (std::vector<std::vector<std::vector<double>) a vector of polygons represented by a vector of x,y coords for the verticies
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
///     robot_radius (double) buffer radius to avoid collisions with the robot body
///     k_nearest (unsigned int) number of neighboring verticies to match to
///     graph_size (unsigned int) number of nodes to use to build the graph
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
#include "roadmap/prm.hpp"
#include "roadmap/utility.hpp"

static std::vector<double> r, g, b;
static double cell_size = 1.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_roadmap");
  ros::NodeHandle n;

  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims;
  std::vector<double> map_y_lims;
  XmlRpc::XmlRpcValue obstacles;
  double robot_radius = 0.0;
  int k_nearest = 5;
  int graph_size = 100;

  n.getParam("obstacles", obstacles);
  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("robot_radius", robot_radius);
  n.getParam("k_nearest", k_nearest);
  n.getParam("graph_size", graph_size);
  n.getParam("cell_size", cell_size);
  n.getParam("r", r);
  n.getParam("g", g);
  n.getParam("b", b);

  std::vector<std::vector<double>> colors;

  for(unsigned int i = 0; i < r.size(); i++)
  {
    r.at(i) /= 255;
    g.at(i) /= 255;
    b.at(i) /= 255;

    colors.push_back({r.at(i), g.at(i), b.at(i)});
  }

  // Build Obstacles vector
  std::vector<std::vector<rigid2d::Vector2D>> polygons;
  rigid2d::Vector2D buf_vec; // for some reason commenting out this line breaks the connection to rigid2d...

  polygons = utility::parse_obstacle_data(obstacles, cell_size);

  // Scale Map Coordinates
  for(unsigned int i = 0; i < map_x_lims.size(); i++)
  {
    map_x_lims.at(i) *= cell_size;
    map_y_lims.at(i) *= cell_size;
  }

  ROS_INFO_STREAM("PRM: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("PRM: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("PRM: k_nearest: " << k_nearest);
  ROS_INFO_STREAM("PRM: graph_size: " << graph_size);
  ROS_INFO_STREAM("PRM: robot_radius: " << robot_radius);
  ROS_INFO_STREAM("PRM: cell size: " << cell_size);
  ROS_INFO_STREAM("PRM: Loaded Params");

  prm::RoadMap prob_road_map(polygons, map_x_lims, map_y_lims);

  prob_road_map.build_map(graph_size, k_nearest, robot_radius);

  const auto all_nodes = prob_road_map.get_nodes();
  const auto all_edges = prob_road_map.get_edges();

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::MarkerArray pub_marks;

  // Put a spherical marker at each node
  for(auto node : all_nodes)
  {
    markers.push_back(utility::make_marker(node, cell_size, colors.at(0)));
  }

  // Draw a line to show all connections.
  for(auto edge : all_edges)
  {
    markers.push_back(utility::make_marker(edge, cell_size, colors.at(2)));
  }

  pub_marks.markers = markers;
  pub_markers.publish(pub_marks);

  ros::spin();
}
