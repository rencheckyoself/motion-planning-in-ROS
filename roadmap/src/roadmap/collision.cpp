/// \file
/// \brief A library containing functions to detect various types of collisions

#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>
#include <iostream>

#include "roadmap/collision.hpp"
#include "rigid2d/rigid2d.hpp"

namespace collision
{
  double point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point)
  {
    // edge vector
    rigid2d::Vector2D s = rigid2d::Vector2D(line_end.x - line_start.x, line_end.y - line_start.y);

    // find the shortest distance from each vertex to the edge
    double c = ((point.x - line_start.x) * (s.x) + (point.y - line_start.y) * (s.y)) / (s.length() * s.length());

    if (c >=0 && c <= 1) // means the vertex is within the bounds of the line segment
    {
      // point on the line closest to the vertex
      rigid2d::Vector2D p = rigid2d::Vector2D(line_start.x + c*s.x, line_start.y + c*s.y);

      return p.distance(point);
    }
    else
    {
      return std::min(point.distance(line_start), point.distance(line_end));
    }
  }

  bool point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point, double threshold)
  {

    double dist = point_to_line_distance(line_start, line_end, point);

    // true if the distance between vertex and line less than the buffer distance
    return dist <= threshold ;
  }

  bool point_inside_convex(rigid2d::Vector2D point, std::vector<rigid2d::Vector2D> polygon, double buffer_radius)
  {

    unsigned int left = 0, right = 0;
    unsigned int poly_size = polygon.size();

    bool collides = true, quit_early = false;

    double min_dist = 10000.0;

    polygon.push_back(polygon.at(0)); // add the first vertex to the end of the list

    // Loop through each vertex
    for(unsigned int i = 0; i < poly_size; i++)
    {
      // Vertex A
      rigid2d::Vector2D a = polygon.at(i);

      // Vertex B
      rigid2d::Vector2D b = polygon.at(i+1);

      // Get direction of perpendicular vector pointing inward to the polygon
      rigid2d::Vector2D u = rigid2d::Vector2D(-(b.y - a.y), b.x - a.x);
      rigid2d::Vector2D n = u.normalize();

      // Vector from vertex A to point
      rigid2d::Vector2D d = rigid2d::Vector2D(point.x - a.x, point.y - a.y);

      // Dot product of n and d
      double r = d.x * n.x + d.y * n.y;

      if(r > 0) right++;
      else if(r < 0) left++;
      else // the point is on the line
      {
        quit_early = true;
        collides = true;
        break;
      }

      min_dist = std::min(point_to_line_distance(a, b, point), min_dist);
    }


    if(left < poly_size && right < poly_size && !quit_early) // this means the point is outside the shape
    {
        if(min_dist > buffer_radius) collides = false;
        else collides = true;
    }
    
    return collides;
  }

  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon)
  {
    return true;
  }

} // end collision
