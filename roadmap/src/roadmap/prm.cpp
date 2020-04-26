/// \file
/// \brief A library for building a Probabilistic Road Map

#include <algorithm>
#include <functional>
#include <iostream>
#include <random>
#include <unordered_set>
#include <vector>

#include "roadmap/prm.hpp"
#include "roadmap/collision.hpp"
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
  // RoadMap CLASS =============================================================
  // ===========================================================================

  RoadMap::RoadMap()
  {
    x_bounds = {0, 10};
    y_bounds = {0, 10};
  }

  RoadMap::RoadMap(std::vector<double> xboundary,std::vector<double> yboundary)
  {
    x_bounds = xboundary;
    y_bounds = yboundary;
  }

  RoadMap::RoadMap(std::vector<std::vector<rigid2d::Vector2D>> polygon_verticies, std::vector<double> xboundary,std::vector<double> yboundary)
  {
    x_bounds = xboundary;
    y_bounds = yboundary;
    obstacles = polygon_verticies;
  }

  void RoadMap::build_map(unsigned int samples, unsigned int k_neighbors, double robot_radius)
  {
    buffer_radius = robot_radius;
    k = k_neighbors;
    n = samples;
    sample_config_space();

    for(auto & node : nodes)
    {
      connect_node(node);
    }
  }

  bool RoadMap::add_node(rigid2d::Vector2D point)
  {
    Node output;

    std::cout << point;

    // check if provided point is valid
    bool valid = node_collisions(point);

    // Add Node to the Road Map or return false
    if(valid)
    {
      output.id = node_cnt;
      output.point = point;
      node_cnt++;
      nodes.push_back(output);

      connect_node(output);

      return true;
    }
    else
    {
      std::cout << "Provided Point is invalid!" << std::endl;
      return false;
    }
  }

  std::vector<Node> RoadMap::get_nodes() const
  {
    return nodes;
  }

  std::vector<Edge> RoadMap::get_edges() const
  {
    return all_edges;
  }

  // PRIVATE MEMBER FUNCTIONS
  void RoadMap::sample_config_space()
  {
    Node buf_node;
    rigid2d::Vector2D buf_point;
    bool valid = true;

    // Sample Configuration space until n valid nodes are created
    while(node_cnt < n)
    {
      buf_point.x = sampleUniformDistribution(x_bounds.at(0) + buffer_radius, x_bounds.at(1) - buffer_radius);
      buf_point.y = sampleUniformDistribution(y_bounds.at(0) + buffer_radius, y_bounds.at(1) - buffer_radius);

      // Check if the node is inside an obstacle
      valid = node_collisions(buf_point);

      // Add Node to the Road Map
      if(valid)
      {
        buf_node.id = node_cnt;
        buf_node.point = buf_point;
        node_cnt++;

        nodes.push_back(buf_node);
      }

    }
  }

  bool RoadMap::node_collisions(rigid2d::Vector2D point)
  {
    bool valid_node = true;

    // Loop through each line segments
    for(auto obstacle : obstacles)
    {
      auto collides = collision::point_inside_convex(point, obstacle, buffer_radius);

      if(collides.at(0))
      {
        valid_node = false;
        break;
      }
    }

    return valid_node;
  }


  void RoadMap::connect_node(Node &node)
  {
    // Find k nearest neighbors
    const auto knn = nearest_neighbors_bf(node);

    // Evaluate each match
    for(auto & qp : knn)
    {
      // If the edge does not exist and if the two nodes are atleast 15cm apart, create an edge
      if(!node.IsConnected(qp.get().id) && qp.get().distance > buffer_radius)
      {

        // Create a temporary edge
        Edge buf_edge;

        buf_edge.edge_id = edge_cnt;

        buf_edge.node1_id = node.id;
        buf_edge.node1 = node.point;

        buf_edge.node2_id = qp.get().id;
        buf_edge.node2 = qp.get().point;

        buf_edge.distance = qp.get().distance;

        // check for path collisions with the obstacles
        bool valid_edge = edge_collisions(buf_edge);

        // check for robot collision with the obstacles

        if(valid_edge)
        {
          all_edges.push_back(buf_edge);

          // add edge to node
          node.edges.push_back(buf_edge);

          // switch 1 and 2 and add to second node
          buf_edge.node1_id = buf_edge.node2_id;
          buf_edge.node1 = buf_edge.node2;
          buf_edge.node2_id = node.id;
          buf_edge.node2 = node.point;

          qp.get().edges.push_back(buf_edge);

          // add connected node ids to the unordered sets
          node.id_set.insert(qp.get().id);
          qp.get().id_set.insert(node.id);

          edge_cnt++;
        }
      }
    }
  }

  std::vector<std::reference_wrapper<Node>> RoadMap::nearest_neighbors_bf(const Node &node)
  {
    // calculate the distance between all nodes
    for(auto & qp : nodes)
    {
      qp.distance = node.point.distance(qp.point);
    }

    // create a vector of references to the elements of the nodes vector
    std::vector<std::reference_wrapper<Node>> input(nodes.begin(), nodes.end());

    // Sort the references by the calculated distance
    std::sort(input.begin(), input.end(), [](std::reference_wrapper<Node> lhs, std::reference_wrapper<Node> rhs) {return lhs.get().distance < rhs.get().distance;});

    std::vector<std::reference_wrapper<Node>> output(input.begin()+1, input.begin()+1+k);

    return output;
  }

  bool RoadMap::edge_collisions(Edge edge)
  {
    bool valid_edge = true;
    bool collides = true;
    // Loop through each obstacle
    for(auto obstacle : obstacles)
    {
      collides = collision::line_shape_intersection(edge.node1, edge.node2, obstacle, buffer_radius);

      if(collides)
      {
        valid_edge = false;
        break;
      }
    }
    return valid_edge;
  }
}
