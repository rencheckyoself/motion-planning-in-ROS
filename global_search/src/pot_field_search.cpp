/// \file
/// \brief Node to plan on the map using potential fields
///
/// PARAMETERS:
///     obstacles (std::vector<std::vector<std::vector<double>) a vector of polygons represented by a vector of x,y coords for the verticies
///     map_x_lims (std::vector<double>) [xmin, xmax] of the map
///     map_y_lims (std::vector<double>) [ymin, ymax] of the map
///     robot_radius (double) buffer radius to avoid collisions with the robot body
///     cell_size (double) distance of each cell
///     r (std::vector<int>) color values
///     g (std::vector<int>) color values
///     b (std::vector<int>) color values
///     start std::vector<double> two double values representing the x,y of the start point
///     goal std::vector<double> two double values representing the x,y of the goal point
/// PUBLISHES:
///     /visualization_marker_array (visualization_msgs::MarkerArray) markers

#include <vector>
#include <algorithm>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "global_search/potential_fields.hpp"
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"
#include "roadmap/utility.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pot_field_search");
  ros::NodeHandle n;

  ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

  std::vector<double> map_x_lims, map_y_lims;
  std::vector<double> start, goal;
  XmlRpc::XmlRpcValue obstacles;
  double robot_radius = 0.0;
  std::vector<double> r, g, b;
  double cell_size = 1.0;
  double att_weight = 1.0, dgstar = 1.0;
  double rep_weight = 1.0, Qstar = 1.0;
  double epsilon = 0.1, zeta = 0.1;

  n.getParam("obstacles", obstacles);
  n.getParam("map_x_lims", map_x_lims);
  n.getParam("map_y_lims", map_y_lims);
  n.getParam("robot_radius", robot_radius);
  n.getParam("cell_size", cell_size);
  n.getParam("att_weight", att_weight);
  n.getParam("dgstar", dgstar);
  n.getParam("rep_weight", rep_weight);
  n.getParam("Qstar", Qstar);
  n.getParam("epsilon", epsilon);
  n.getParam("zeta", zeta);
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

  ROS_INFO_STREAM("PFSRCH: x_lims: " << map_x_lims.at(0) << ", " << map_x_lims.at(1));
  ROS_INFO_STREAM("PFSRCH: y_lims: " << map_y_lims.at(0) << ", " << map_y_lims.at(1));
  ROS_INFO_STREAM("PFSRCH: robot_radius: " << robot_radius);
  ROS_INFO_STREAM("PFSRCH: cell size: " << cell_size);
  ROS_INFO_STREAM("PFSRCH: att weight: " << att_weight);
  ROS_INFO_STREAM("PFSRCH: dg*: " << dgstar);
  ROS_INFO_STREAM("PFSRCH: rep weight: " << rep_weight);
  ROS_INFO_STREAM("PFSRCH: Q*: " << Qstar);
  ROS_INFO_STREAM("PFSRCH: epsilon: " << epsilon);
  ROS_INFO_STREAM("PFSRCH: zeta: " << zeta);
  ROS_INFO_STREAM("PFSRCH: start coordinate: " << start_pt);
  ROS_INFO_STREAM("PFSRCH: goal coordinate: " << goal_pt);
  ROS_INFO_STREAM("PFSRCH: Loaded Params");


  grid::Map map(polygons, map_x_lims, map_y_lims);

  // Configure Potential Field
  pfield::PtField pot_field_search(map, goal_pt, zeta, att_weight, dgstar, rep_weight, Qstar);

  std::vector<rigid2d::Vector2D> path = {start_pt};

  auto cur_pos = start_pt;

  std::vector<visualization_msgs::Marker> markers;
  visualization_msgs::MarkerArray pub_marks;

  prm::Node start_node, goal_node;

  start_node.point = start_pt;
  goal_node.point = goal_pt;
  goal_node.id = 1;

  // Draw Start and Goal
  markers.push_back(utility::make_marker(start_node, cell_size*2, std::vector<double>({0, 1, 0}))); // start
  markers.push_back(utility::make_marker(goal_node, cell_size*2, std::vector<double>({1, 0, 0}))); // goal

  ros::Rate frames(1);

  ros::Duration(3).sleep(); // wait for rviz

  // Search for the path while the robot is beyond the termination threshold
  while(cur_pos.distance(goal_pt) > epsilon && ros::ok())
  {
    ROS_INFO_STREAM("Distance to Go: " << cur_pos.distance(goal_pt));

    cur_pos = pot_field_search.PlanOneStep(cur_pos);

    path.push_back(cur_pos);

    // Draw path
    for(auto it = path.begin(); it < path.end()-1; it++)
    {
      markers.push_back(utility::make_marker(*it, *(it+1), it-path.begin(), cell_size, colors.at(4)));
    }

    pub_marks.markers = markers;
    pub_markers.publish(pub_marks);

    markers.clear();

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}
