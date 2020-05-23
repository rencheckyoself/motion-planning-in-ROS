#ifndef PFIELD_INCLUDE_GUARD_HPP
#define PFIELD_INCLUDE_GUARD_HPP
/// \file
/// \brief A library to plan based on a potential field

#define BIG_NUM 10000.0

#include "rigid2d/rigid2d.hpp"
#include "roadmap/grid.hpp"

namespace pfield
{
  /// brief class to allow planning
  class PtField
  {
  public:

    /// \brief constuctor to initialize the Potential Field Planner
    /// \param map the known Map
    /// \param z the step size for gradient descent
    /// \param aw weighting factor the attactive component
    /// \param dg threshold for piecewise attractive gradient calculation
    /// \param rw weighting factor the repulsive component
    /// \param Qs threshold for obstacles range of influence
    PtField(grid::Map map, rigid2d::Vector2D goal, double z, double aw, double dg, double rw, double Qs);

    /// \brief given the robots location, plan its next step
    /// \param cur_loc the current location of the robot
    /// \returns the next location to move to
    rigid2d::Vector2D PlanOneStep(rigid2d::Vector2D cur_loc);

    /// \brief retrieve the whole planned path
    /// \returns the planned path
    std::vector<rigid2d::Vector2D> get_path();

  private:
    grid::Map known_map; ///< the known map

    rigid2d::Vector2D goal_loc; ///< the goal location

    double zeta; ///< the step size for gradient descent

    double att_weight; ///< weighting factor the attactive component
    double dg_star; ///< threshold for piecewise attractive gradient calculation

    double rep_weight; ///< weighting factor the repulsive component
    double Qstar; ///< threshold for obstacles range of influence

    std::vector<rigid2d::Vector2D> final_path; ///< final path determined

    /// \brief Calculate the attractive component of the "force" based on the distance to goal
    /// \param cur_loc the current location of the robot
    /// \returns the total attrative gradient
    rigid2d::Vector2D calculate_u_att(rigid2d::Vector2D cur_loc);

    /// \brief Calculate the repulsive component of the "force" based on the distance to each obstacle
    /// \param cur_loc the current location of the robot
    /// \returns the total repulsive gradient
    rigid2d::Vector2D calculate_u_rep(rigid2d::Vector2D cur_loc);

    /// \brief Calculate the repulsive component due to a given obstacle
    /// \param polygon a set of points that define a convex polygon
    /// \param cur_loc the current location of the robot
    /// \returns a vector of the repulsice gradient components for the provided obstacle
    rigid2d::Vector2D u_rep_component(std::vector<rigid2d::Vector2D> polygon, rigid2d::Vector2D cur_loc);
  };
}

#endif //PFIELD_INCLUDE_GUARD_HPP
