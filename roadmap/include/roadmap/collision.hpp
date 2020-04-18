#ifndef COLLISIONS_INCLUDE_GUARD_HPP
#define COLLISIONS_INCLUDE_GUARD_HPP
/// \file
/// \brief A library containing functions to detect various types of collisions

#include "rigid2d/rigid2d.hpp"

namespace collision
{

  /// \brief Determine if a point is within a certain distance to a line
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param point the point to calculate the distance for
  /// \param threshold the distance threshold to compare against
  /// \returns True if the distance between the point and the line is LESS THAN the provided threshold
  bool point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point, double threshold);

  /// \brief Determine if a point is within a certain distance to a line
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param point the point to calculate the distance for
  /// \returns The shortest distance between the point and the line if the point is within the line segment or the minimum between the distances to the line_start or line_end
  double point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point);

  /// \brief Determine if a point is inside of a convex polygon, the points must be provided in order either cw or ccw.
  /// This function will account for connected the last vertex to the first vertex by appending the first vertex
  /// to the back of the vector.
  /// \param point the point to analyze
  /// \param polygon a vector of verticies that define the polygon
  /// \param buffer_radius a buffer distance to incorporate to the polygon boundary
  /// \returns True if the point is inside the polygon or inside of the buffer zone of the polygon
  bool point_inside_convex(rigid2d::Vector2D point, std::vector<rigid2d::Vector2D> polygon, double buffer_radius);

  /// \brief Determine if a line segment intersects a convex polygon
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param polygon a vector of verticies that define the polygon
  /// \returns True if there is an intersection between the line segment and polygon
  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon);
}

#endif
