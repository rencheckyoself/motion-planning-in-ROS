#ifndef GRID_INCLUDE_GUARD_HPP
#define GRID_INCLUDE_GUARD_HPP
/// \file
/// \brief A library for building an occupancy grid

#include <vector>
#include <unordered_set>

#include "rigid2d/rigid2d.hpp"

namespace grid
{
  class Grid
  {

  public:

    /// \brief Initialization to construct a grid in an empty 10x10 area
    ///
    Grid();

    /// \brief Initialization to construct a grid in an empty user defined area
    /// \param xboundary a 2 element vector defining the map's x bounds
    /// \param yboundary a 2 element vector defining the map's y bounds
    /// \param samples the number of nodes for the road map
    Grid(std::vector<double> xboundary,std::vector<double> yboundary);

    /// \brief Initialization to construct a grid in a user defined area with obstacles
    /// \param polygon_verticies a vector of vectors defining the verticies of each obstacle in order going counter-clockwise
    /// \param xboundary a 2 element vector defining the map's x bounds
    /// \param yboundary a 2 element vector defining the map's y bounds
    Grid(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary, std::vector<double> yboundary);

    /// \brief Wrapper function to call all nessissary functions to build the grid
    /// \param grid_res the distance for the cell length/height in meters
    /// \param robot_radius the radius to use as a buffer around the robot for collision detection
    void build_grid(double grid_res, double robot_radius);

    /// \brief convert the grid data into a in row major order
    /// \returns grid occupancy data in row major order
    std::vector<int> get_grid();

  private:
    std::vector<std::vector<rigid2d::Vector2D>> obstacles; // obstacles in the map
    std::vector<double> x_bounds; // x bounds of the map
    std::vector<double> y_bounds; // y bounds of the map

    double buffer_radius = 0; // buffer distance to incorporate when detecting collisions
    double grid_res = 1; // meters per grid cell

    std::vector<std::vector<int>> occ_data; // occupancy grid data, 0 is free, 50 is buffer zone, 100 is occupied

    /// brief convert from grid coordinates (integers) to world coordinates (meters)
    /// param grid_coord grid location to convert
    /// returns matching world coordinate
    rigid2d::Vector2D grid_to_world(rigid2d::Vector2D grid_coord);

    /// brief convert from world coordinates to grid coordinates
    /// param world_coord world location to convert
    /// returns matching grid coordinate
    rigid2d::Vector2D world_to_grid(rigid2d::Vector2D world_coord);

    /// brief determine which cells are occupied based on the stored obstacles
    ///
    void get_occupied_cells();

    /// brief determine which cells are occupied based on the buffer area around the obstacles and map boarder
    ///
    void get_bufferzone_cells();
  };
}

#endif // GRID_INCLUDE_GUARD_HPP
