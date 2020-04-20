/// \file
/// \brief A library for building an occupied grid

#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

#include "roadmap/grid.hpp"
#include "roadmap/collision.hpp"
#include "rigid2d/rigid2d.hpp"

namespace grid
{

  Grid::Grid()
  {
    x_bounds = {0, 10};
    y_bounds = {0, 10};
  }

  Grid::Grid(std::vector<double> xboundary,std::vector<double> yboundary)
  {
    x_bounds = xboundary;
    y_bounds = yboundary;
  }

  Grid::Grid(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary, std::vector<double> yboundary)
  {

  }

  void Grid::build_grid(double grid_res, double robot_radius){}

  std::vector<int> Grid::get_grid(){}

}
