
#include <vector>
#include <iostream>
#include <XmlRpcValue.h>
#include "rigid2d/rigid2d.hpp"
#include "roadmap/utility.hpp"


namespace utility
{
  std::vector<std::vector<rigid2d::Vector2D>> parse_obstacle_data(XmlRpc::XmlRpcValue obstacles, double cell_size)
  {
    // Build Obstacles vector
    std::vector<std::vector<rigid2d::Vector2D>> polygons;
    std::vector<rigid2d::Vector2D> buf_poly;
    rigid2d::Vector2D buf_vec;

    for(int i=0; i < obstacles.size(); i++) // loop through each obstacle
    {
      for(int j=0; j < obstacles[i].size(); j++) // loop through each point in the obstacle and scale coordinates
      {
        buf_vec.x = double(obstacles[i][j][0]) * cell_size;
        buf_vec.y = double(obstacles[i][j][1]) * cell_size;

        buf_poly.push_back(buf_vec);
      }

      std::cout << "Points in polygon " << i << "\n";
      for(auto poly : buf_poly)
      {
        std::cout << "(" << poly.x << ", " << poly.y << ")\t"  ;
      }
      std::cout << "\n";

      polygons.push_back(buf_poly);
      buf_poly.clear();
    }

    return polygons;
  }

  std::vector<rigid2d::Vector2D> create_map_vector(std::vector<double> map_x_lims, std::vector<double> map_y_lims)
  {
    rigid2d::Vector2D buf_point;
    std::vector<rigid2d::Vector2D> map_boarder;

    buf_point.x = map_x_lims.at(0);
    buf_point.y = map_y_lims.at(0);
    map_boarder.push_back(buf_point);

    buf_point.x = map_x_lims.at(1);
    map_boarder.push_back(buf_point);

    buf_point.y = map_y_lims.at(1);
    map_boarder.push_back(buf_point);

    buf_point.x = map_x_lims.at(0);
    map_boarder.push_back(buf_point);

    return map_boarder;
  }

  geometry_msgs::Point Vec2D_to_GeoPt(rigid2d::Vector2D vec)
  {
    geometry_msgs::Point point;

    point.x = vec.x;
    point.y = vec.y;
    point.z = 0;

    return point;
  }

  nav_msgs::OccupancyGrid make_grid_msg(grid::Grid *grid, double cell_size, double res)
  {
    auto occ_grid = grid->get_grid_flatten();
    auto grid_dims = grid->get_grid_dimensions();

    nav_msgs::OccupancyGrid occ_msg;

    occ_msg.header.frame_id = "map";
    occ_msg.header.stamp = ros::Time::now();

    occ_msg.info.map_load_time = ros::Time::now();
    occ_msg.info.resolution = cell_size/res;
    occ_msg.info.height = grid_dims.at(1);
    occ_msg.info.width = grid_dims.at(0);

    occ_msg.info.origin.position.x = 0;
    occ_msg.info.origin.position.y = 0;
    occ_msg.info.origin.position.z = 0;
    occ_msg.info.origin.orientation.x = 0;
    occ_msg.info.origin.orientation.y = 0;
    occ_msg.info.origin.orientation.z = 0;
    occ_msg.info.origin.orientation.w = 1;

    occ_msg.data = occ_grid;

    return occ_msg;
  }

  visualization_msgs::Marker make_marker(std::vector<rigid2d::Vector2D> point_list, double scale, std::vector<double> color)
  {
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> geo_points;

    // Convert the points
    for(const auto p : point_list)
    {
      geo_points.push_back(Vec2D_to_GeoPt(p));
    }

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "Expanded Nodes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.points = geo_points;

    marker.scale.x = 0.3 * scale;
    marker.scale.y = 0.3 * scale;

    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }

  visualization_msgs::Marker make_marker(prm::Node node, double scale, std::vector<double> color)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "Nodes";
    marker.id = node.id;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = node.point.x;
    marker.pose.position.y = node.point.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.3 * scale;
    marker.scale.y = 0.3 * scale;
    marker.scale.z = 0.3 * scale;

    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }

  visualization_msgs::Marker make_marker(prm::Edge edge, double scale, std::vector<double> color)
  {
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> points = {Vec2D_to_GeoPt(edge.node1), Vec2D_to_GeoPt(edge.node2)};

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "Edges";
    marker.id = edge.edge_id;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.points = points;

    marker.scale.x = 0.1 * scale;

    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }

  visualization_msgs::Marker make_marker(rigid2d::Vector2D pt1, rigid2d::Vector2D pt2, int marker_id, double scale, std::vector<double> color)
  {
    visualization_msgs::Marker marker;

    std::vector<geometry_msgs::Point> points = {Vec2D_to_GeoPt(pt1), Vec2D_to_GeoPt(pt2)};

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "Path";
    marker.id = marker_id;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.z = 0.03;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.points = points;

    marker.scale.x = 0.1 * scale;

    marker.color.r = color.at(0);
    marker.color.g = color.at(1);
    marker.color.b = color.at(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }
}
