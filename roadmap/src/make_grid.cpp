/// \file
/// \brief Node to create and draw a grid
///
/// PARAMETERS:
///     obstacles (std::vector<std::vector<std::vector<double>) a vector of polygons represented by a vector of x,y coords for the verticies
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
///     robot_radius (double) buffer radius to avoid collisions with the robot body
///     cell_size (double) scaling factor for the map
///     grid_res (double) scaling factor for the grid cell size
/// PUBLISHES:
///     /visualization_marker_array (visualization_msgs::MarkerArray) markers

#include <vector>
#include <algorithm>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include "nav_msgs/OccupancyGrid.h"

#include "rigid2d/rigid2d.hpp"
#include "roadmap/grid.hpp"
#include "roadmap/utility.hpp"

static std::vector<double> r, g, b;
static double cell_size = 1.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_roadmap");
  ros::NodeHandle n;

  ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("grip_map", 1, true);
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims;
  std::vector<double> map_y_lims;
  XmlRpc::XmlRpcValue obstacles;
  double robot_radius = 0.0;
  int grid_res = 1;

  n.getParam("obstacles", obstacles);
  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("robot_radius", robot_radius);
  n.getParam("cell_size", cell_size);
  n.getParam("grid_res", grid_res);
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
  ROS_INFO_STREAM("GRID: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("GRID: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));

  ROS_INFO_STREAM("GRID: cell size: " << cell_size);

  if(grid_res < 1) ROS_FATAL_STREAM("GRID: Tried grid res: " << grid_res <<". Grid resolution must be >= 1" );
  else ROS_INFO_STREAM("GRID: grid res: " << grid_res);


  // Build Obstacles vector
  std::vector<std::vector<rigid2d::Vector2D>> polygons;
  rigid2d::Vector2D buf_vec; // for some reason commenting out this line breaks the connection to rigid2d...

  polygons = utility::parse_obstacle_data(obstacles, 1);

  // Initialize Grid
  // grid::Grid grid_world(polygons, map_x_lims, map_y_lims);
  grid::Grid grid_world(map_x_lims, map_y_lims);


  grid_world.build_grid(cell_size, grid_res, robot_radius);

  auto occ_msg = utility::make_grid_msg(&grid_world, cell_size, grid_res);
  pub_map.publish(occ_msg);

  // Uncomment to also display the graph corresponding to the grid //
  // grid_world.generate_centers_graph();
  // auto all_nodes = grid_world.get_nodes();
  // auto all_edges = grid_world.get_edges();
  //
  // std::vector<visualization_msgs::Marker> markers;
  // visualization_msgs::MarkerArray pub_marks;
  //
  // // Put a spherical marker at each node
  // for(auto node : all_nodes)
  // {
  //   markers.push_back(utility::make_marker(node, cell_size, colors.at(0)));
  // }
  //
  // // Draw a line to show all connections.
  // for(auto edge : all_edges)
  // {
  //   markers.push_back(utility::make_marker(edge, cell_size, colors.at(2)));
  // }
  //

  // pub_marks.markers = markers;
  // pub_markers.publish(pub_marks);

  ros::spin();
}
