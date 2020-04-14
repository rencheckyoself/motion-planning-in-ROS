#ifndef PRM_INCLUDE_GUARD_HPP
#define PRM_INCLUDE_GUARD_HPP
/// \file
/// \brief A library for building a Probabilistic Road Map

#include <vector>
#include<unordered_set>

#include "rigid2d/rigid2d.hpp"

namespace prm
{
  struct Edge
  {
    int edge_id = -1;

    int node1_id = -1;
    rigid2d::Vector2D node1;

    int node2_id = -1;
    rigid2d::Vector2D node2;

    double distance; // length of the edge
  };

  struct Node
  {
    int id = -1; // unique node ID
    rigid2d::Vector2D point; // x,y location relative to the world
    std::vector<Edge> edges; // edges connecting to nearby nodes
    std::unordered_set<int> id_set; // nodes that should be connected
    double weight = 0; // weight to determine if map is properly sampled

    double distance=0; // the distance to a node, used to find k-nearest nodes

    /// \brief Determine if a node is connected to this one
    /// \param node_id ID to check for a connection
    /// \returns True if a connection exists
    bool IsConnected(int node_id) const
    {
      const auto search = id_set.find(node_id);
      return (search == id_set.end()) ? false : true;
    }

    /// \brief Compares the distance value of this node object and another
    /// \param rhs pointer to another node object
    /// \returns True if this node has a smaller distance
    bool operator<(const Node &rhs) const
    {
      return (distance < rhs.distance);
    }
  };

  class RoadMap
  {
  public:

    /// \breif Initialization to construct a road map in an empty 10x10 area with 100 samples
    RoadMap();

    /// \breif Initialization to construct a road map in an empty user defined area
    /// \param xboundary a 2 element vector defining the map's x bounds
    /// \param yboundary a 2 element vector defining the map's y bounds
    /// \param samples the number of nodes for the road map
    RoadMap(std::vector<double> xboundary,std::vector<double> yboundary);

    /// \breif Initialization to construct a road map in a user defined area with obstacles
    /// \param polygon_verticies a vector of vectors defining the verticies of each obstacle in order going counter-clockwise
    /// \param xboundary a 2 element vector defining the map's x bounds
    /// \param yboundary a 2 element vector defining the map's y bounds
    RoadMap(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary, std::vector<double> yboundary);

    /// \brief Wrapper function to call all nessissary functions to build the PRM
    /// \param samples the number of nodes for the road map
    /// \param k_neighbors the amount of neighbors to try and create an edge to
    /// \param robot_radius the radius to use as a buffer around the robot for collision detection
    void build_map(unsigned int samples, unsigned int k_neighbors, double robot_radius);

    /// \brief Wrapper function to get the vector of nodes
    /// \returns the full node vector
    std::vector<Node> get_nodes() const;

    /// \brief Wrapper function to get the vector of unique edges
    /// \returns the full edge vector
    std::vector<Edge> get_edges() const;

  private:
    std::vector<std::vector<rigid2d::Vector2D>> obstacles;
    std::vector<double> x_bounds; // x bounds of the map
    std::vector<double> y_bounds; // y bounds of the map

    std::vector<Node> nodes; // all nodes in the road map
    std::vector<Edge> all_edges; // all edges in the road map

    double buffer_radius = 0; // buffer distance to incorporate when detecting collisions

    unsigned int n = 100; // number of nodes in the map
    unsigned int k = 10; // number of nearest neighbors to find

    /// \brief Randomly Sample the configuration space to retrieve a set of nodes for the roadmap
    ///
    void sample_config_space();

    /// \brief Determine if the node was sampled from an area inside an obstacle
    /// \param point the configuration of a new potential node
    /// \returns true if the node is valid
    bool node_collisions(rigid2d::Vector2D point);

    /// \brief Find nodes that are near each other and connect them if possible.
    ///
    void connect_nodes();

    /// \brief wrapper for all of the edge collision methods
    /// \param edge the edge to compare against all polygons
    /// \returns true if the edge is valid
    bool edge_collisions(Edge edge);

    // /// \brief determine if an edge intersects a polygon
    // /// \param edge the edge to compare against all polygons
    // /// \param obstacle the polgon to check
    // /// \returns true if there was a collision
    // bool line_intersections(Edge edge, std::vector<rigid2d::Vector2D> obstacle) const;

    /// \brief Brute force k-nearest search
    /// \param reference to a node object to base the search on
    /// \returns a vector of references the k-nearest nodes to the input node
    std::vector<std::reference_wrapper<Node>> nearest_neighbors_bf(const Node &node);
  };
}


#endif
