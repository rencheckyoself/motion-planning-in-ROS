/// \file
/// \brief Node to create, draw, and plan on a probabilistic road map
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

static std::vector<double> r, g, b;
static double cell_size = 1.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prm_search");
  ros::NodeHandle n;

  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims, map_y_lims;
  std::vector<double> start, goal;
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

  // convert start/goal to vector2D
  rigid2d::Vector2D start_pt(start.at(0) * cell_size, start.at(1) * cell_size);
  rigid2d::Vector2D goal_pt(goal.at(0) * cell_size, goal.at(1) * cell_size);

  ROS_INFO_STREAM("PRMSRCH: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("PRMSRCH: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("PRMSRCH: k_nearest: " << k_nearest);
  ROS_INFO_STREAM("PRMSRCH: graph_size: " << graph_size);
  ROS_INFO_STREAM("PRMSRCH: robot_radius: " << robot_radius);
  ROS_INFO_STREAM("PRMSRCH: cell size: " << cell_size);
  ROS_INFO_STREAM("PRMSRCH: start coordinate: " << start_pt);
  ROS_INFO_STREAM("PRMSRCH: goal coordinate: " << goal_pt);
  ROS_INFO_STREAM("PRMSRCH: Loaded Params");

  // Create the PRM
  prm::RoadMap prob_road_map(polygons, map_x_lims, map_y_lims);
  prob_road_map.build_map(graph_size, k_nearest, robot_radius);

  bool resultS, resultG = 0;

  // Add in the start and goal
  resultS = prob_road_map.add_node(start_pt);
  resultG = prob_road_map.add_node(goal_pt);

  if(!resultS)
  {
    ROS_FATAL_STREAM("PRMSRCH: Invalid start node. \n Given start: " << start_pt);
    ros::shutdown();
  }
  else if(!resultG)
  {
    ROS_FATAL_STREAM("PRMSRCH: Invalid goal node. \n Given goal: " << goal_pt);
    ros::shutdown();
  }

  // Retrieve the PRM
  auto all_nodes = prob_road_map.get_nodes();
  auto all_edges = prob_road_map.get_edges();
  unsigned int node_cnt = all_nodes.size();

  prm::Node start_node = all_nodes.at(node_cnt-2);
  prm::Node goal_node = all_nodes.at(node_cnt-1);

  grid::Map map(polygons, map_x_lims, map_y_lims);

  // Configure the A* and Theta* searches
  hsearch::AStar a_star_search(&all_nodes);

  hsearch::ThetaStar t_star_search(&all_nodes, map, robot_radius);

  // conduct A* search
  bool search_result_astar = a_star_search.ComputeShortestPath(start_node, goal_node);
  ROS_INFO_STREAM("PRMSRCH: A* Search Complete!\n");

  // conduct Theta* search
  bool search_result_tstar = t_star_search.ComputeShortestPath(start_node, goal_node);
  ROS_INFO_STREAM("PRMSRCH: Theta* Search Complete!\n");

  // Check for failure
  if(!search_result_astar)
  {
    ROS_FATAL_STREAM("PRMSRCH: A* Search failed to find a path given the start and goal.\n");
    // ros::shutdown();
  }
  else if(!search_result_tstar)
  {
    ROS_FATAL_STREAM("PRMSRCH: Theta* Search failed to find a path given the start and goal.\n");
    // ros::shutdown();
  }

  // Retrieve the path
  std::vector<rigid2d::Vector2D> a_path = a_star_search.get_path();
  std::vector<rigid2d::Vector2D> t_path = t_star_search.get_path();

  ROS_INFO_STREAM("PRMSRCH: A* Path has " << a_path.size() << " nodes.");
  ROS_INFO_STREAM("PRMSRCH: Theta* Path has " << t_path.size() << " nodes.");

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::MarkerArray pub_marks;

  // Put a spherical marker at each node
  for(auto it = all_nodes.begin(); it < all_nodes.end()-2; it++)
  {
    markers.push_back(utility::make_marker(*it, cell_size, colors.at(0)));
  }

  // Draw a line to show all connections.
  for(auto edge : all_edges)
  {
    markers.push_back(utility::make_marker(edge, cell_size/2, colors.at(2)));
  }

  // Draw Start and Goal
  markers.push_back(utility::make_marker(start_node, cell_size*2, std::vector<double>({0, 1, 0}))); // start
  markers.push_back(utility::make_marker(goal_node, cell_size*2, std::vector<double>({1, 0, 0}))); // goal

  // Draw A* path
  for(auto it = a_path.begin(); it < a_path.end()-1; it++)
  {
    markers.push_back(utility::make_marker(*it, *(it+1), it-a_path.begin(), cell_size, std::vector<double>({0, 0, 0})));
  }

  auto start_id = a_path.size();

  // Draw Theta* path
  for(auto it = t_path.begin(); it < t_path.end()-1; it++)
  {
    markers.push_back(utility::make_marker(*it, *(it+1), start_id + (it-t_path.begin()), cell_size, colors.at(4)));
  }

  pub_marks.markers = markers;
  pub_markers.publish(pub_marks);

  ros::spin();
}
