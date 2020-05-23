/// \file
/// \brief A library to plan based on a potential field

#include <vector>

#include "global_search/potential_fields.hpp"
#include "rigid2d/rigid2d.hpp"
#include "roadmap/grid.hpp"

namespace pfield
{

  PtField::PtField(grid::Map map, rigid2d::Vector2D goal, double z, double aw, double dg, double rw, double Qs)
  {
    known_map = map;
    goal_loc = goal;

    zeta = z;

    att_weight = aw;
    dg_star = dg;

    rep_weight = rw;
    Qstar = Qs;
  }

  rigid2d::Vector2D PtField::PlanOneStep(rigid2d::Vector2D cur_loc)
  {
    rigid2d::Vector2D next_loc = cur_loc;

    // caculate the attractive gradient
    rigid2d::Vector2D att_grad = calculate_u_att(cur_loc);

    // calculate the repulsive gradient
    rigid2d::Vector2D rep_grad(0,0);

    auto tot_grad = att_grad + rep_grad;

    auto D_n = tot_grad.normalize();

    next_loc = cur_loc - zeta*D_n;

    return next_loc;
  }

  std::vector<rigid2d::Vector2D> PtField::get_path()
  {
    return final_path;
  }

  rigid2d::Vector2D PtField::calculate_u_att(rigid2d::Vector2D cur_loc)
  {
    double dist = cur_loc.distance(goal_loc);
    rigid2d::Vector2D att_grad = (cur_loc - goal_loc)*att_weight;

    // If outside the threshold, use the conic well, otherwise stick with the parabolic
    if(dist > dg_star)
    {
      att_grad = (dg_star * att_grad)/dist;
    }

    return att_grad;
  }

}
