/// \file
/// \brief A library for building an occupied grid

#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

#include "roadmap/collision.hpp"
#include "roadmap/grid.hpp"
#include "roadmap/prm.hpp"
#include "roadmap/utility.hpp"
#include "rigid2d/rigid2d.hpp"

namespace grid
{

  Map::Map(std::vector<std::vector<rigid2d::Vector2D>> obs, std::vector<double> x, std::vector<double> y)
  {
    obstacles = obs;
    x_bounds = x;
    y_bounds = y;

    map_vector = utility::create_map_vector(x_bounds, y_bounds);
  }

  // ===========================================================================
  // Grid CLASS ================================================================
  // ===========================================================================

  Grid::Grid()
  {
    og_map.x_bounds = {0, 10};
    og_map.y_bounds = {0, 10};

    og_map.map_vector = utility::create_map_vector(og_map.x_bounds, og_map.y_bounds);

  }

  Grid::Grid(std::vector<double> xboundary,std::vector<double> yboundary)
  {
    og_map.x_bounds = xboundary;
    og_map.y_bounds = yboundary;

    og_map.map_vector = utility::create_map_vector(og_map.x_bounds, og_map.y_bounds);
  }

  Grid::Grid(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary, std::vector<double> yboundary)
  {
    og_map.x_bounds = xboundary;
    og_map.y_bounds = yboundary;
    og_map.obstacles = polygon_verticies;

    og_map.map_vector = utility::create_map_vector(og_map.x_bounds, og_map.y_bounds);
  }

  void Grid::build_grid(double cell_size, unsigned int grid_res, double buffer_radius)
  {
    this->cell_size = cell_size;
    this->grid_res = grid_res;
    this->buffer_radius = buffer_radius;

    // update the map dimensions and obstacle coordinates based on the grid resolution
    grid_resize();

    // scale buffer radius to the integer grid size and add half of cell diagonal to stay conservative
    double grid_buffer = (buffer_radius/cell_size)*grid_res + (std::sqrt(2) * 0.5);

    rigid2d::Vector2D cell_center;

    int occupied = 100, in_buffer = 50, free = 0;

    std::vector<signed char> grid_row;

    // Loop through each cell on the grid and determine if the center is inside a polygon or inside the buffer zone
    for(int i = 0; i < grid_dimensions.at(1); i++) // y coord
    {
      for(int j = 0; j < grid_dimensions.at(0); j++) // x coord
      {

        cell_center.x = j + 0.5;
        cell_center.y = i + 0.5;

        grid_row.push_back(free); // add the cell to the array

        std::vector<bool> occ_result = {false, false}; // assume no collision

        for(auto obstacle : scaled_map.obstacles)
        {
          occ_result = collision::point_inside_convex(cell_center, obstacle, grid_buffer);

          if(occ_result.at(0) && occ_result.at(1)) // if the point is inside the obstacle
          {
            grid_row.at(j) = occupied;
            break;
          }
          else if(occ_result.at(0) && !occ_result.at(1)) // if the point is outside the obstacle, but in the buffer zone
          {
            if(buffer_radius == 0) grid_row.at(j) = occupied; // apply the correct color
            else grid_row.at(j) = in_buffer;
          }
        }

        // if there was no obstacle collision and a buffer has been set, check the map boarder
        if(!occ_result.at(0) && buffer_radius != 0)
        {
          auto boarder_res = cell_near_boarder(cell_center, grid_buffer);
          if(boarder_res) grid_row.at(j) = in_buffer;
        }
      }

      // add the row to the 2D list
      occ_data.push_back(grid_row);
      grid_row.clear();
    }
  }

  void Grid::generate_centers_graph()
  {

    int cnt= 0;

    nodes.clear();
    all_edges.clear();

    // Loop through each cell on the grid and create the node corresponding to it
    for(int i = 0; i < grid_dimensions.at(1); i++) // y coord
    {
      std::vector<prm::Node> row;
      for(int j = 0; j < grid_dimensions.at(0); j++) // x coord
      {
        rigid2d::Vector2D grid_coord(j,i);

        rigid2d::Vector2D world_coord = grid_to_world(grid_coord);

        prm::Node buf;

        buf.id = cnt;
        buf.point = world_coord;

        row.push_back(buf);

        cnt++;
      }
      nodes.push_back(row);
    }

    cnt = 0;

    // Loop through each node and create edges to the 8 neighbors connections
    for(int i = 0; i < grid_dimensions.at(1); i++) // y coord
    {
      for(int j = 0; j < grid_dimensions.at(0); j++) // x coord
      {

        prm::Node& node = nodes.at(i).at(j);

        // Check the 8 Neighboring Nodes
        for(int m = -1; m < 2; m++) //y shift
        {
          for(int n = -1; n < 2; n++) //x shift
          {
            // skip over the 0 shift and any shift that is out of bounds
            if(m == 0 && n == 0) continue;
            else if((j + n) < 0 || (j + n) >= grid_dimensions.at(0)) continue;
            else if((i + m) < 0 || (i + m) >= grid_dimensions.at(1)) continue;
            else
            {
              prm::Node& neighbor = nodes.at(i+m).at(j+n); // get the reference to the neighbor node

              if(!node.IsConnected(neighbor.id))
              {
                prm::Edge buf_edge;

                buf_edge.edge_id = cnt;

                buf_edge.node1_id = node.id;
                buf_edge.node1 = node.point;
                buf_edge.node2_id = neighbor.id;
                buf_edge.node2 = neighbor.point;

                buf_edge.distance = node.point.distance(neighbor.point);

                // add to main list
                all_edges.push_back(buf_edge);

                // Add to Node
                node.edges.push_back(buf_edge);
                node.id_set.insert(neighbor.id);

                // switch 1 and 2 and add to second node
                buf_edge.node1_id = buf_edge.node2_id;
                buf_edge.node1 = buf_edge.node2;
                buf_edge.node2_id = node.id;
                buf_edge.node2 = node.point;

                neighbor.edges.push_back(buf_edge);
                neighbor.id_set.insert(node.id);

                cnt++;
              }
            }
          }
        }
      }
    }
  }

  bool Grid::update_grid(std::vector<std::pair<rigid2d::Vector2D, signed char>> points)
  {
    bool output = false;

    for (const auto & point : points)
    {
      const auto cur = occ_data.at(point.first.y).at(point.first.x);

      if(cur != point.second)
      {
        occ_data.at(point.first.y).at(point.first.x) = point.second;

        if(cur == 0 && point.second != 0) output = true;
        else if(cur !=0 && point.second == 0) output = true;
      }
    }
    return output;
  }

  std::vector<std::vector<prm::Node>> Grid::get_nodes() const
  {
    return nodes;
  }

  std::vector<prm::Node> Grid::get_nodes_flatten() const
  {
    // collapse 2d vector into row major order
    std::vector<prm::Node> output;

    for(const auto row : nodes)
    {
      for(const auto point : row)
      {
        output.push_back(point);
      }
    }

    return output;
  }

  std::vector<prm::Edge> Grid::get_edges() const
  {
    return all_edges;
  }

  std::vector<std::vector<signed char>> Grid::get_grid() const
  {
    return occ_data;
  }

  std::vector<signed char> Grid::get_grid_flatten() const
  {
    // collapse 2d vector into row major order
    std::vector<signed char> output;

    for(const auto row : occ_data)
    {
      for(const auto point : row)
      {
        output.push_back(point);
      }
    }

    return output;
  }

  std::vector<int> Grid::get_grid_dimensions() const
  {
    return grid_dimensions;
  }

  rigid2d::Vector2D Grid::grid_to_world(rigid2d::Vector2D grid_coord)
  {
    double ratio = cell_size/ static_cast<double>(grid_res);
    double shift = 0.5;

    return rigid2d::Vector2D((grid_coord.x + shift) * ratio, (grid_coord.y + shift) * ratio);
  }

  rigid2d::Vector2D Grid::world_to_grid(rigid2d::Vector2D world_coord)
  {
    double ratio = cell_size/ static_cast<double>(grid_res);

    double shift = 0.5;

    return rigid2d::Vector2D(std::round((world_coord.x/ratio) - shift), std::round((world_coord.y/ratio) - shift));
  }

  // Private Functions =========================================================

  void Grid::grid_resize()
  {
    scaled_map = og_map;

    // scale the map
    std::for_each(scaled_map.x_bounds.begin(), scaled_map.x_bounds.end(), [&](double &n){ n *= grid_res;});
    std::for_each(scaled_map.y_bounds.begin(), scaled_map.y_bounds.end(), [&](double &n){ n *= grid_res;});

    std::for_each(scaled_map.map_vector.begin(), scaled_map.map_vector.end(), [&](rigid2d::Vector2D &n){ n.x *= grid_res; n.y *= grid_res;});

    for(auto & obstacle : scaled_map.obstacles)
    {
      std::for_each(obstacle.begin(), obstacle.end(), [&](rigid2d::Vector2D &n){ n.x *= grid_res; n.y *= grid_res;});
    }

    // get grid dimensions
    grid_dimensions.push_back(scaled_map.x_bounds.at(1) - scaled_map.x_bounds.at(0));
    grid_dimensions.push_back(scaled_map.y_bounds.at(1) - scaled_map.y_bounds.at(0));
  }

  bool Grid::cell_near_boarder(rigid2d::Vector2D center, double threshold)
  {
    bool collides = false;

    auto boarder = scaled_map.map_vector;

    boarder.push_back(scaled_map.map_vector.at(0));

    for(unsigned int i = 0; i < scaled_map.map_vector.size(); i++)
    {
      collides = collision::point_to_line_distance(boarder.at(i), boarder.at(i+1), center, threshold);
      if(collides) return collides;
    }

    return collides;
  }

}
