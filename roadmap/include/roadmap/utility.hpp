#ifndef UTILITY_INCLUDE_GUARD_HPP
#define UTILITY_INCLUDE_GUARD_HPP
/// \file
/// \brief A library of utility functions for the various nodes and libraries of this package

#include <vector>
#include <XmlRpcValue.h>
#include "rigid2d/rigid2d.hpp"

namespace utility
{

  /// \brief converts obstacle data from a YAML file into a vector of vectors. TODO: changes this to a template based output
  /// \param obstacles the a list of lists containing obstacle vertices
  /// \param cell_size a scaling factor to apply to the vertex coordinates. Use 1 if you do not want to scale them.
  /// \returns a vector of vectors containing obstacle vertices
  std::vector<std::vector<rigid2d::Vector2D>> parse_obstacle_data(XmlRpc::XmlRpcValue obstacles, double cell_size);
}

#endif // UTILITY_INCLUDE_GAURD_HPP
