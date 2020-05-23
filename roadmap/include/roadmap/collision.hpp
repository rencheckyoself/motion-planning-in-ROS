#ifndef COLLISIONS_INCLUDE_GUARD_HPP
#define COLLISIONS_INCLUDE_GUARD_HPP
/// \file
/// \brief A library containing functions to detect various types of collisions

#include "rigid2d/rigid2d.hpp"

namespace collision
{

  /// \brief Used to return information from the point to line distance function
  struct DistRes
  {
    bool inside_segment = false; ///< True if the point lies inbetween the line segment bounds, False otherwise.
    double distance = 0; ///< Min distance from the point to the linesegment. If the point is not within the line segment, the distance to the closest vertex
    rigid2d::Vector2D point; ///< the x,y coordinates of the closest point on the line or vertex

    /// \brief default constructor - initialize all to zeros
    DistRes();

    /// \brief C  reate the result of the point to line distance function
    /// \param status True if the point is within the line segment bounds
    /// \param dist Min distance from the point to the linesegment. If the point is not within the line segment, the distance to the closest vertex
    /// \param p point that correlates to the stored distance
    DistRes(bool status, double dist, rigid2d::Vector2D p);
  };

  /// \brief Determine if a point is within a certain distance to a line
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param point the point to calculate the distance for
  /// \param threshold the distance threshold to compare against
  /// \returns True if the distance between the point and the line is LESS THAN the provided threshold
  bool point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point, double threshold);

  /// \brief Calculate the minimum distance to a line segment
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param point the point to calculate the distance for
  /// \returns The shortest distance between the point and the line if the point is within the line segment or the minimum between the distances to the line_start or line_end
  DistRes point_to_line_distance(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, rigid2d::Vector2D point);

  /// \brief Determine if a point is inside of a convex polygon, the points must be provided in order either cw or ccw.
  /// This function will account for connected the last vertex to the first vertex by appending the first vertex
  /// to the back of the vector.
  /// \param point the point to analyze
  /// \param polygon a vector of verticies that define the polygon in order, either cw or ccw.
  /// \param buffer_radius a buffer distance to incorporate to the polygon boundary
  /// \returns a 2 element vector, first element is true if there is a collision, second element describes the cause: True for inside the shape, False for outside the shape but in the buffer zone.
  std::vector<bool> point_inside_convex(rigid2d::Vector2D point, std::vector<rigid2d::Vector2D> polygon, double buffer_radius);

  /// \brief Determine if a line segment intersects a convex polygon
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param polygon a vector of verticies that define the polygon in ccw order
  /// \returns True if there is an intersection between the line segment and polygon
  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon);

  /// \brief Determine if a line segment intersects a convex polygon or comes within a certain distance of it. This assumes the line start and end points are
  /// known to be outside of the buffer area of the polygon
  /// \param line_start the point of the beginning of the line segment
  /// \param line_end the point of the end of the line segment
  /// \param polygon a vector of verticies that define the polygon in ccw order
  /// \param buffer_radius a buffer distance to incorporate to the polygon boundary
  /// \returns True if there is an intersection between the line segment and polygon or if the minimum distance to the line and shape is less than the buffer radius
  bool line_shape_intersection(rigid2d::Vector2D line_start, rigid2d::Vector2D line_end, std::vector<rigid2d::Vector2D> polygon, double buffer_radius);
}

#endif
