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
    int child_id = -1; // node id for the connected node
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
    RoadMap(std::vector<double> xboundary,std::vector<double> yboundary, unsigned int samples);

    /// \breif Initialization to construct a road map in a user defined area with obstacles
    /// \param polygon_verticies a vector of vectors defining the verticies of each obstacle
    /// \param xboundary a 2 element vector defining the map's x bounds
    /// \param yboundary a 2 element vector defining the map's y bounds
    /// \param samples the number of nodes for the road map
    RoadMap(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary, std::vector<double> yboundary, unsigned int samples);

    /// \brief Wrapper function to call all nessissary functions to build the PRM
    ///
    void build_map();

    std::vector<Node> get_nodes();

  private:
    std::vector<std::vector<rigid2d::Vector2D>> obstacles;
    std::vector<double> x_bounds; // x bounds of the map
    std::vector<double> y_bounds; // y bounds of the map

    std::vector<Node> nodes; // all nodes in the road map

    unsigned int n = 100; // number of nodes in the map
    unsigned int k = 10; // number of nearest neighbors to find

    /// \brief Randomly Sample the configuration space to retrieve a set of nodes for the roadmap
    ///
    void sample_config_space();

    /// \brief Find nodes that are near each other and connect them if possible.
    ///
    void connect_nodes();

    /// \brief Brute force k-nearest search
    /// \param reference to a node object to base the search on
    /// \returns a vector of the k-nearest nodes to the input node
    std::vector<Node> nearest_neighbors_bf(const Node &node);
  };
}


#endif
