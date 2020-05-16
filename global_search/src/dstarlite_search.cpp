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
///     start std::vector<double> two double values representing the x,y of the start point
///     goal std::vector<double> two double values representing the x,y of the goal point
///     sensor_range (double) double value representing the range of a simulated sensor fixed to the center of the robot
/// PUBLISHES:
///     /visualization_marker_array (visualization_msgs::MarkerArray) markers
///     /grid_map (nav_msgs::OccupancyGrid) occupancy data

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
  ros::init(argc, argv, "dstarlite_search");
  ros::NodeHandle n;

  ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("grip_map", 2);
  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 2);

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::MarkerArray pub_marks;

  std::vector<double> map_x_lims, map_y_lims;
  std::vector<double> start, goal;
  XmlRpc::XmlRpcValue obstacles;
  double robot_radius = 0.0;
  int grid_res = 1;
  std::vector<double> r, g, b;
  double cell_size = 1.0;
  double sensor_range = cell_size*3;

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
  n.getParam("sensor_range", sensor_range);

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

  // convert the sensor range to grid cells
  int sensor_range_grid = std::ceil((sensor_range * grid_res) / cell_size);

  if(sensor_range_grid < 2)
  {
    ROS_WARN_STREAM("GRDSRCH: Provdied sensor range rounded up to 2 cells.");
    sensor_range_grid = 2;
  }
  else ROS_INFO_STREAM("GDSRCH: Sensor Range: " << sensor_range_grid);

  // Build Obstacles vector
  std::vector<std::vector<rigid2d::Vector2D>> polygons;
  rigid2d::Vector2D buf_vec; // for some reason commenting out this line breaks the connection to rigid2d...

  polygons = utility::parse_obstacle_data(obstacles, 1);

  // Initialize Grid that represents the fully known map
  grid::Grid grid_world(polygons, map_x_lims, map_y_lims);
  grid_world.build_grid(cell_size, grid_res, robot_radius);
  grid_world.generate_centers_graph();

  // Initialize an empty grid of free cells
  grid::Grid free_grid(map_x_lims, map_y_lims);
  free_grid.build_grid(cell_size, grid_res, robot_radius);
  free_grid.generate_centers_graph();

  auto grid_graph = free_grid.get_nodes();
  auto grid_dims = free_grid.get_grid_dimensions();

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
  hsearch::DStarLite dsl_search(&grid_graph, &free_grid, start_pt, goal_pt);

  // Buffer variables to save all the markers to detele/update
  std::vector<visualization_msgs::Marker> path_markers;
  visualization_msgs::Marker exp_nodes;

  auto start_node = free_grid.get_nodes().at(start_pt.y).at(start_pt.x);
  auto goal_node = free_grid.get_nodes().at(goal_pt.y).at(goal_pt.x);

  rigid2d::Vector2D robot_pos = start_node.point; // set the robot position with the corrrect world coordinates

  ros::Rate frames(2);

  bool new_info = true;

  auto known_occ = grid_world.get_grid();

  frames.sleep(); // short pause to give rviz to load

  auto occ_msg = utility::make_grid_msg(&free_grid, cell_size, grid_res);
  pub_map.publish(occ_msg);


  std::vector<rigid2d::Vector2D> traversed_path;
  std::vector<rigid2d::Vector2D> dsl_path;
  std::vector<rigid2d::Vector2D> dsl_expands;

  traversed_path.push_back(robot_pos);

  // Start loop
  while(ros::ok())
  {
    // std::cout << i << "\tNew Info?: " << new_info << "\n";

    // If there is new map information, update the path as needed
    if(new_info)
    {
      // Plan path
      bool dsl_res = dsl_search.ComputeShortestPath();
      ROS_INFO_STREAM("GDSRCH: Search Complete!\n");

      // Check for failure
      if(!dsl_res)
      {
        ROS_FATAL_STREAM("GDSRCH: D* Lite Search failed to find a path for the current map configuration.\n");
      }

      // retrieve results
      dsl_path = dsl_search.get_path();
      dsl_expands = dsl_search.get_expanded_nodes();

      ROS_INFO_STREAM("DSLSRCH: D* Lite Path has " << dsl_path.size() << " nodes.");

      // Draw Expanded Nodes
      exp_nodes = utility::make_marker(dsl_expands, cell_size/grid_res, colors.at(2));
      markers.push_back(exp_nodes);

      std::reverse(dsl_path.begin(), dsl_path.end());

      if(dsl_path.back() != robot_pos)
      {
        ROS_FATAL_STREAM("The robot has teleported between path searches!");
        robot_pos = dsl_path.back();
      }
    }

    new_info = false;

    // VIZUALIZE THE RESULTS

    // Draw the robot
    markers.push_back(utility::make_marker(robot_pos, cell_size*2, std::vector<double>({0, 0, 1})));

    // Draw Start and Goal
    markers.push_back(utility::make_marker(start_node, cell_size*2, std::vector<double>({0, 1, 0})));
    markers.push_back(utility::make_marker(goal_node, cell_size*2, std::vector<double>({1, 0, 0})));

    // Draw D* Lite path
    for(auto it = dsl_path.begin(); it < dsl_path.end()-1; it++)
    {
      visualization_msgs::Marker buf = utility::make_marker(*it, *(it+1), it-dsl_path.begin(), cell_size, colors.at(4));
      path_markers.push_back(buf);
      markers.push_back(buf);
    }

    // Draw the path that has been traversed by the robot

    ROS_INFO_STREAM(traversed_path.size());
    traversed_path.push_back(robot_pos);
    if(traversed_path.size() > 1)
    {
      for(auto it = traversed_path.begin(); it < traversed_path.end()-1; it++)
      {
        int ns_id = (it-traversed_path.begin());

        visualization_msgs::Marker buf = utility::make_marker(*it, *(it+1), ns_id, cell_size, std::vector<double>({0, 0, 0}), "Trav");
        path_markers.push_back(buf);
        markers.push_back(buf);
      }
    }

    // clear expanded nodes if needed
    if(dsl_expands.empty())
    {
      exp_nodes.action = visualization_msgs::Marker::DELETE;
      markers.push_back(exp_nodes);
    }

    pub_marks.markers = markers;
    pub_markers.publish(pub_marks);

    // vizualize map
    auto occ_msg = utility::make_grid_msg(&free_grid, cell_size, grid_res);
    pub_map.publish(occ_msg);

    ros::spinOnce();

    // sleep til next loop
    frames.sleep();

    // If the robot has not reached the goal yet, move and update the map
    if(dsl_path.size() > 1)
    {
      // traversed_path.push_back(robot_pos);
      dsl_path.pop_back();
      robot_pos = dsl_path.back();

      // Check for map updates -- Update based on the sensor range param
      std::vector<std::pair<rigid2d::Vector2D, signed char>> map_update;

      rigid2d::Vector2D robot_grid = free_grid.world_to_grid(robot_pos);

      // Simulate a sensor by extracting information from the known grid
      for(int j = -sensor_range_grid; j < sensor_range_grid; j++)
      {
        int yi = robot_grid.y+j;

        if(yi < 0 || yi >= grid_dims.at(1)) continue;

        for(int k = -sensor_range_grid; k < sensor_range_grid; k++)
        {

          int xi = robot_grid.x+k;
          if(xi < 0 || xi >= grid_dims.at(0)) continue;

          map_update.push_back(std::make_pair(rigid2d::Vector2D(xi, yi), known_occ.at(yi).at(xi)));
        }
      }

      ROS_INFO_STREAM("Updating " << map_update.size() << " Cells.");

      // Inform LPA* of the "sensor" readings and update all of the effected verticies
      if(!map_update.empty())
      {
        dsl_search.UpdateRobotLoc(robot_grid);
        new_info = dsl_search.MapChange(map_update);
      }
    }

    // Reset the paths
    markers.clear();
    for(auto & marker : path_markers)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      markers.push_back(marker);
    }

    pub_marks.markers = markers;
    pub_markers.publish(pub_marks);

    markers.clear();
    dsl_expands.clear();
    path_markers.clear();
  }
}
