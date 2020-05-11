/// \file
/// \brief Node to create, draw, and plan on a changing grid map
///
/// PARAMETERS:
///     obstacles (std::vector<std::vector<std::vector<double>) a vector of polygons represented by a vector of x,y coords for the verticies
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
///     robot_radius (double) buffer radius to avoid collisions with the robot body
///     cell_size (double) scaling factor for the map
///     grid_res (double) scaling factor for the grid cell size
///     r (std::vector<int>) color values
///     g (std::vector<int>) color values
///     b (std::vector<int>) color values

#include <vector>
#include <algorithm>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "global_search/heuristic_search.hpp"
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"
#include "roadmap/utility.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_iter_search");
  ros::NodeHandle n;

  ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("grip_map", 1, true);
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims, map_y_lims;
  std::vector<double> start, goal;
  XmlRpc::XmlRpcValue obstacles;
  double robot_radius = 0.0;
  int grid_res = 1;
  std::vector<double> r, g, b;
  double cell_size = 1.0;

  n.getParam("obstacles", obstacles);
  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("robot_radius", robot_radius);
  n.getParam("cell_size", cell_size);
  n.getParam("grid_res", grid_res);
  n.getParam("r", r);
  n.getParam("g", g);
  n.getParam("b", b);

  n.getParam("start", start);
  n.getParam("goal", goal);

  std::vector<std::vector<double>> colors;

  for(unsigned int i = 0; i < r.size(); i++)
  {
    r.at(i) /= 255;
    g.at(i) /= 255;
    b.at(i) /= 255;

    colors.push_back({r.at(i), g.at(i), b.at(i)});
  }

  if(grid_res < 1)
  {
    ROS_FATAL_STREAM("GDSRCH: Tried grid res: " << grid_res <<". Grid resolution must be >= 1. Using default resolution of 1.");
    grid_res = 1;
  }
  else ROS_INFO_STREAM("GDSRCH: grid res: " << grid_res);

  // Build Obstacles vector
  std::vector<std::vector<rigid2d::Vector2D>> polygons;
  rigid2d::Vector2D buf_vec; // for some reason commenting out this line breaks the connection to rigid2d...

  polygons = utility::parse_obstacle_data(obstacles, 1);

  // Initialize Grid that represents the fully known map
  grid::Grid grid_world(polygons, map_x_lims, map_y_lims);
  grid_world.build_grid(cell_size, grid_res, robot_radius);
  grid_world.generate_centers_graph();

  auto grid_graph = grid_world.get_nodes();
  auto grid_graph_flat = grid_world.get_nodes_flatten();
  auto grid_dims = grid_world.get_grid_dimensions();

  // Initialize an empty grid of free cells
  grid::Grid free_grid(map_x_lims, map_y_lims);
  free_grid.build_grid(cell_size, grid_res, robot_radius);
  free_grid.generate_centers_graph();

  // auto grid_graph = free_grid.get_nodes();
  // auto grid_graph_flat = free_grid.get_nodes_flatten();
  // auto grid_dims = free_grid.get_grid_dimensions();

  // convert start/goal to vector2D
  rigid2d::Vector2D start_pt(start.at(0) * grid_res, start.at(1) * grid_res);
  rigid2d::Vector2D goal_pt(goal.at(0) * grid_res, goal.at(1) * grid_res);

  ROS_INFO_STREAM("GDSRCH: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("GDSRCH: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("GDSRCH: robot_radius: " << robot_radius);
  ROS_INFO_STREAM("GDSRCH: cell size: " << cell_size);
  ROS_INFO_STREAM("GDSRCH: start coordinate: " << start_pt);
  ROS_INFO_STREAM("GDSRCH: goal coordinate: " << goal_pt);
  ROS_INFO_STREAM("GDSRCH: Loaded Params");

  // Check to make sure start and goal are valid
  if(grid_world.get_grid().at(start_pt.y).at(start_pt.x) != 0)
  {
    ROS_FATAL_STREAM("GDSRCH: Start Point is located in an occupied cell\n\tWorld Coords: " << grid_world.grid_to_world(start_pt));
    ros::shutdown();
  }

  if(grid_world.get_grid().at(goal_pt.y).at(goal_pt.x) != 0)
  {
    ROS_FATAL_STREAM("GDSRCH: Goal Point is located in an occupied cell\n\tWorld Coords: " << grid_world.grid_to_world(goal_pt));
    ros::shutdown();
  }

  // Initialize the search on the empty map
  hsearch::LPAStar lpa_search(&grid_graph, &grid_world, start_pt, goal_pt);

  // Do an intial pass at the plan
  bool lpa_res = lpa_search.ComputeShortestPath();
  ROS_INFO_STREAM("GDSRCH: Search Complete!\n");

  // Check for failure
  if(!lpa_res)
  {
    ROS_FATAL_STREAM("GDSRCH: LPA* Search failed to find a path for the current map configuration.\n");
  }


  // Retrieve the path
  std::vector<rigid2d::Vector2D> lpa_path = lpa_search.get_path();

  // get analysis info
  auto lpa_expands = lpa_search.get_expanded_nodes();

  ROS_INFO_STREAM("GDSRCH: LPA* Path has " << lpa_path.size() << " nodes.");

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::MarkerArray pub_marks;

  auto start_node = free_grid.get_nodes().at(start_pt.y).at(start_pt.x);
  auto goal_node = free_grid.get_nodes().at(goal_pt.y).at(goal_pt.x);

  // Draw Start and Goal
  markers.push_back(utility::make_marker(start_node, cell_size*2, std::vector<double>({0, 1, 0}))); // start
  markers.push_back(utility::make_marker(goal_node, cell_size*2, std::vector<double>({1, 0, 0}))); // goal

  // Draw LPA* path
  for(auto it = lpa_path.begin(); it < lpa_path.end()-1; it++)
  {
    markers.push_back(utility::make_marker(*it, *(it+1), it-lpa_path.begin(), cell_size, std::vector<double>({0, 0, 0})));
  }

  // Draw Expanded Nodes
  markers.push_back(utility::make_marker(lpa_expands, cell_size, std::vector<double>({0, 0, 1})));

  auto occ_msg = utility::make_grid_msg(&grid_world, cell_size, grid_res);
  pub_map.publish(occ_msg);

  pub_marks.markers = markers;
  pub_markers.publish(pub_marks);

  ros::spin();
}
