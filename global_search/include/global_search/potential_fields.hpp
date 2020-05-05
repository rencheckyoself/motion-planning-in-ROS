#ifndef PFIELD_INCLUDE_GUARD_HPP
#define PFIELD_INCLUDE_GUARD_HPP
/// \file
/// \brief A library to plan based on a potential field

#include "rigid2d/rigid2d.hpp"
#include "roadmap/grid.hpp"

namespace pfield
{
  struct Pot_Grid
  {
    rigid2d::Vector2D point;
    double U;
    bool free = true;
  };

  class PtField
  {
  public:

    /// \brief Calculate the attractive component of the "force" based on the distance to goal
    calculate_u_att();



  private:
    grid::Grid og_grid;


  }
}

#endif //PFIELD_INCLUDE_GUARD_HPP
