/// \file
/// \brief A library containing functions to detect various types of collisions

#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

#include "roadmap/collision.hpp"
#include "rigid2d/rigid2d.hpp"

namespace collision
{
  DistRes::DistRes()
  {
    inside_segment = false;
    distance = 0;
    point = rigid2d::Vector2D(0,0);
  }

  DistRes::DistRes(bool status, double dist, rigid2d::Vector2D p)
  {
    inside_segment = status;
    distance = dist;
    point = p;
  }

  DistRes point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point)
  {
    // edge vector
    rigid2d::Vector2D s = rigid2d::Vector2D(line_end.x - line_start.x, line_end.y - line_start.y);

    // find the shortest distance from each vertex to the edge
    double c = ((point.x - line_start.x) * (s.x) + (point.y - line_start.y) * (s.y)) / (s.length() * s.length());

    if (c >=0 && c <= 1) // means the vertex is within the bounds of the line segment
    {
      // point on the line closest to the vertex
      rigid2d::Vector2D p = rigid2d::Vector2D(line_start.x + c*s.x, line_start.y + c*s.y);

      return DistRes(true, p.distance(point), p);
    }
    else
    {
      // starting point is closer
      if(point.distance(line_start) < point.distance(line_end))
      {
          return DistRes(false, point.distance(line_start), line_start);
      }
      else
      {
        return DistRes(false, point.distance(line_end), line_end);
      }
    }
  }

  bool point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point, double threshold)
  {

    auto dist = point_to_line_distance(line_start, line_end, point);

    // true if the distance between vertex and line less than the buffer distance
    return dist.distance <= threshold ;
  }

  std::vector<bool> point_inside_convex(rigid2d::Vector2D point, std::vector<rigid2d::Vector2D> polygon, double buffer_radius)
  {
    unsigned int left = 0, right = 0;
    unsigned int poly_size = polygon.size();

    bool quit_early = false;

    double min_dist = 10000.0;

    polygon.push_back(polygon.at(0)); // add the first vertex to the end of the list

    // first element is the collision information, second is the cause.
    std::vector<bool> output = {true, true};

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

      auto calc_distance = point_to_line_distance(a, b, point);

      if(r > 0) right++;
      else if(r < 0) left++;
      else if(r == 0 && calc_distance.inside_segment) // the point is on the line
      {
        // std::cout << "Point is on the line. \n";
        quit_early = true;
        break;
      }

      min_dist = std::min(calc_distance.distance, min_dist);
    }

    if(left < poly_size && right < poly_size && !quit_early) // this means the point is outside the shape
    {
        if(min_dist > buffer_radius)
        {
          // std::cout << "Point is OUTSIDE of the shape and OUTSIDE the buffer.\n";
          output.at(0) = false;
        }
        else
        {
          // std::cout << "Point is OUTSIDE of the shape but INSIDE the buffer. \n";
          output.at(0) = true;
          output.at(1) = false;
        }
    }

    return output;
  }

  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon)
  {
    polygon.push_back(polygon.at(0)); // add the first vertex to the end of the list
    bool collides = true;

    double t_e = 0.0, t_l = 1.0;

    // Loop through each line segment
    for(unsigned int i = 0; i < polygon.size()-1; i++)
    {
      // Vertex A
      rigid2d::Vector2D a = polygon.at(i);

      // Vertex B
      rigid2d::Vector2D b = polygon.at(i+1);

      // Get perpendicular vector pointing outward to the polygon
      rigid2d::Vector2D u = rigid2d::Vector2D(b.y - a.y, -(b.x - a.x));
      rigid2d::Vector2D n = u.normalize();

      // edge vector
      rigid2d::Vector2D s = rigid2d::Vector2D(line_end.x - line_start.x, line_end.y - line_start.y);

      // vector between edge and obstalce line starts
      rigid2d::Vector2D p0_vi = rigid2d::Vector2D(line_start.x - a.x, line_start.y - a.y);

      double num = - n.dot(p0_vi);
      double den = n.dot(s);

      if(den == 0) continue; // Test for paralellism between the edge and obstacle line segment

      double t = num/den;

      if(den < 0) {t_e = std::max(t_e, t);} // segment is potentially entering the polygon
      else {t_l = std::min(t_l, t);} // segment is potentially leaving the polygon

      if(t_l < t_e)
      {
        collides = false; // means the edge cannot intersect the polygon
        break;
      }
    }

    return collides;
  }

  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon, double buffer_radius)
  {
    polygon.push_back(polygon.at(0)); // add the first vertex to the end of the list
    bool collides = true;

    bool intsec_test = true;

    double t_e = 0.0, t_l = 1.0;

    // Loop through each line segment
    for(unsigned int i = 0; i < polygon.size()-1; i++)
    {
      // Vertex A
      rigid2d::Vector2D a = polygon.at(i);

      // Vertex B
      rigid2d::Vector2D b = polygon.at(i+1);

      // Get perpendicular vector pointing outward to the polygon
      rigid2d::Vector2D u = rigid2d::Vector2D(b.y - a.y, -(b.x - a.x));
      rigid2d::Vector2D n = u.normalize();

      // edge vector
      rigid2d::Vector2D s = rigid2d::Vector2D(line_end.x - line_start.x, line_end.y - line_start.y);

      // vector between edge and obstalce line starts
      rigid2d::Vector2D p0_vi = rigid2d::Vector2D(line_start.x - a.x, line_start.y - a.y);

      double num = - n.dot(p0_vi);
      double den = n.dot(s);

      if(den == 0)  // Test for paralellism between the edge and obstacle line segment
      {
        // Shift the s vector slightly to break parallelism
        s += rigid2d::Vector2D(line_end.x/1000.0, line_end.y/1000.0);
        den = n.dot(s);
      }

      double t = num/den;

      if(den < 0) {t_e = std::max(t_e, t);} // segment is potentially entering the polygon
      else {t_l = std::min(t_l, t);} // segment is potentially leaving the polygon

      if(t_l < t_e && intsec_test) // means the edge cannot intersect a convex polygon
      {
        collides = false;
        intsec_test = false;
      }

      // check the line enters the buffer radius
      if(point_to_line_distance(line_start, line_end, a, buffer_radius))
      {
        collides = true;
        break;
      }
    }

    return collides;
  }
} // end collision
