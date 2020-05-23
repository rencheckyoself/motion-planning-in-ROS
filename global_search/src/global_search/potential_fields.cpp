/// \file
/// \brief A library to plan based on a potential field

#include <vector>

#include "global_search/potential_fields.hpp"
#include "rigid2d/rigid2d.hpp"
#include "roadmap/collision.hpp"
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
    rigid2d::Vector2D rep_grad = calculate_u_rep(cur_loc);

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

  rigid2d::Vector2D PtField::calculate_u_rep(rigid2d::Vector2D cur_loc)
  {
    rigid2d::Vector2D rep_grad;

    // loop through each obstacle and find the distance to the nearest point
    for(auto obstacle : known_map.obstacles)
    {
      rep_grad += u_rep_component(obstacle, cur_loc);
    }

    // Also check the map boarder
    rep_grad += u_rep_component(known_map.map_vector, cur_loc);

    return rep_grad;
  }

  rigid2d::Vector2D PtField::u_rep_component(std::vector<rigid2d::Vector2D> polygon, rigid2d::Vector2D cur_loc)
  {
    // add the first vertex to the end to close the loop of line segments
    polygon.push_back(polygon.at(0));

    collision::DistRes min_loc;
    min_loc.distance = BIG_NUM;

    collision::DistRes res;

    // calculate the distance to each line segment and save the minimum distance
    for(auto it = polygon.begin(); it < polygon.end()-1; it++)
    {
      res = collision::point_to_line_distance(*it, *(it+1), cur_loc);

      if(res.distance < min_loc.distance) min_loc = res;
    }

    rigid2d::Vector2D buf(0,0);

    // calculate the component of the repulsive gradient based on the range of influence
    if(min_loc.distance < Qstar)
    {
      // First get the vector that goes from the point on obstacle to the cur_loc
      auto unit_distvec = (cur_loc - res.point).normalize();

      // calculate the gradient
      buf = rep_weight * ((1.0/Qstar) - (1.0/min_loc.distance)) * (1.0/(min_loc.distance*min_loc.distance)) * unit_distvec;
    }

    return buf;
  }
}
