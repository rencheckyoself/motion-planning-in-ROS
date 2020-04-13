/// \file
/// \brief A library for building a Probabilistic Road Map

#include <algorithm>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

#include "roadmap/prm.hpp"
#include "rigid2d/rigid2d.hpp"

namespace prm
{

  /// \brief Create a twister for random sampling
  /// \returns a twister for random sampling
  static std::mt19937 & get_random()
  {
      // static variables inside a function are created once and persist for the remainder of the program
      static std::random_device rd{};
      static std::mt19937 mt{rd()};
      // we return a reference to the pseudo-random number genrator object. This is always the
      // same object every time get_random is called
      return mt;
  }

  /// \brief Sample a random value between the provided limits
  /// \param llim the lower bound of the range
  /// \param ulim the upper bound of the range
  /// \returns a random value
  double sampleUniformDistribution(double llim, double ulim)
  {
    std::uniform_real_distribution<> d(llim, ulim);
    return d(get_random());
  }

  // ===========================================================================
  // RoadMap CLASS =================================================================
  // ===========================================================================

  RoadMap::RoadMap()
  {
    x_bounds = {0, 10};
    y_bounds = {0, 10};
    n = 100;
  }

  RoadMap::RoadMap(std::vector<double> xboundary,std::vector<double> yboundary, unsigned int samples)
  {
    n = samples;
    x_bounds = xboundary;
    y_bounds = yboundary;
  }

  RoadMap::RoadMap(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary,std::vector<double> yboundary, unsigned int samples)
  {
    n = samples;
    x_bounds = xboundary;
    y_bounds = yboundary;
    obstacles = polygon_verticies;
  }

  void RoadMap::build_map()
  {
    sample_config_space();
    connect_nodes();
  }

  std::vector<Node> RoadMap::get_nodes()
  {
    return nodes;
  }

  // PRIVATE MEMBER FUNCTIONS
  void RoadMap::sample_config_space()
  {
    unsigned int cnt = 0;
    Node buf_node;
    rigid2d::Vector2D buf_point;
    bool valid = true;

    // Sample Configuration space until n valid nodes are created
    while(cnt < n)
    {
      buf_point.x = sampleUniformDistribution(x_bounds.at(0), x_bounds.at(1));
      buf_point.y = sampleUniformDistribution(y_bounds.at(0), y_bounds.at(1));

      // add collision check here

      // Add Node to the Road Map
      if(valid)
      {
        buf_node.id = cnt;
        buf_node.point = buf_point;
        cnt++;

        nodes.push_back(buf_node);
      }
    }
  }

  void RoadMap::connect_nodes()
  {
    for(auto node : nodes)
    {
      // Find k nearest neighbors
      std::vector<Node> knn = nearest_neighbors_bf(node);

      std::cout << "Matches: " << knn.size() << "\n";

      // Add Edges where appropriate
      for(auto qp : knn)
      {
        // If the edge does not exist and if the two nodes are atleast 15cm apart, create an edge
        if(!node.IsConnected(qp.id) && qp.distance < 0.15)
        {

          bool collision_occured = false;

          // check for path collisions with the obstacles

          // check for robot collision with the obstacles

          if(!collision_occured)
          {
            // Create the edge
            Edge buf_edge;
            buf_edge.child_id = qp.id;
            buf_edge.distance = qp.distance;

            node.edges.push_back(buf_edge); // add edge to the node
            node.id_set.insert(qp.id); // add connected node id to the unordered set
          }
        }
      }

      std::cout << "Edges Created: " << node.edges.size() << "\n";
    }
  }

  std::vector<Node> RoadMap::nearest_neighbors_bf(const Node &node)
  {
    std::vector<Node> input = nodes;

    // calculate the distance between all nodes
    for(auto qp : input)
    {
      qp.distance = node.point.distance(qp.point);
    }

    // Sort nodes by the calculated distance
    std::sort(input.begin(), input.end());

    std::vector<Node> output(input.begin()+1, input.begin()+k);

    return output;
  }

}
