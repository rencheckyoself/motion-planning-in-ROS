
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
}
