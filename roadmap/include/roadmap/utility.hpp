#ifndef UTILITY_INCLUDE_GUARD_HPP
#define UTILITY_INCLUDE_GUARD_HPP
/// \file
/// \brief A library of utility functions for the various nodes and libraries of this package

#include <vector>

#include <XmlRpcValue.h>

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"


namespace utility
{



  /// \brief converts obstacle data from a YAML file into a vector of vectors. TODO: changes this to a template based output
  /// \param obstacles the a list of lists containing obstacle vertices
  /// \param cell_size a scaling factor to apply to the vertex coordinates. Use 1 if you do not want to scale them.
  /// \returns a vector of vectors containing obstacle vertices
  std::vector<std::vector<rigid2d::Vector2D>> parse_obstacle_data(XmlRpc::XmlRpcValue obstacles, double cell_size);

  /// \brief gerenate a vector of the 4 map verticies
  /// \param map_x_lims the x axis limits of the map
  /// \param map_y_lims the y axis limits of the map
  /// \returns a vector of the map verticies in ccw order
  std::vector<rigid2d::Vector2D> create_map_vector(std::vector<double> map_x_lims, std::vector<double> map_y_lims);

  /// \brief Convert a Vector2D into a geometry_msgs/Point
  /// \param vec point represented with a 2D vector
  /// \returns a point represented with a geometry_msgs/Point
  geometry_msgs::Point Vec2D_to_GeoPt(rigid2d::Vector2D vec);

  /// \brief Create a spherical Marker based on a node struct
  /// \param node a node struct to vizualize
  /// \param scale the amount to scale the preset marker size (should be graph cell size)
  /// \param color the r,g,b color values in a vector
  /// \returns a marker to add to the MarkerArray
  visualization_msgs::Marker make_marker(prm::Node node, double scale, std::vector<double> color);

  /// \brief Create a line Marker based on an edge struct
  /// \param edge an edge struct to vizualize
  /// \param scale the amount to scale the preset marker size (should be graph cell size)
  /// \param color the r,g,b color values in a vector
  /// \returns a marker to add to the MarkerArray
  visualization_msgs::Marker make_marker(prm::Edge edge, double scale, std::vector<double> color);
}

#endif // UTILITY_INCLUDE_GAURD_HPP
