#ifndef HSEARCH_INCLUDE_GUARD_HPP
#define HSEARCH_INCLUDE_GUARD_HPP
/// \file
/// \brief A library containing classes to perform various types of search algorithms

#include <memory>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"

namespace hsearch
{

  // Used to track if a node is on the open or closed list
  enum status {New, Open, Closed};

  struct SearchNode
  {
    std::shared_ptr<prm::Node> node_p; ///< contains info about ID, location and, neighbors

    int search_id = 0;

    double f_val; ///< total node cost
    double g_val; ///< path cost from start to this
    double h_val; ///< estimated cost from this to goal

    std::shared_ptr<prm::Node> parent_p; ///< the parent of this node

    status state; ///< current status of the node

    SearchNode() {};

    /// \brief Create a new start node
    SearchNode(const prm::Node & n);

    /// \brief Custom function used for proper sorting of the open list.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is less than rhs
    bool operator<(const SearchNode &rhs) const;

    /// \brief Custom function used for proper sorting of the open list.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is greater than rhs
    bool operator>(const SearchNode &rhs) const;

    /// \brief Custom function to evaluate node equality.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is the same as rhs
    // bool operator==(const SearchNode &rhs) const;
  };

  /// \brief Overload the cout operator to print the info in a SearchNode
  /// \param os the output stream
  /// \param n a SearchNode reference
  /// \returns an output stream
  std::ostream & operator<<(std::ostream & os, const SearchNode & n);

  /// \brief the Path 1 cost between a node and its neighbor
  /// \param s the current node being expanded
  /// \param sp the neighbor node being evaluated
  double path1_cost(SearchNode s, SearchNode sp);

  /// \brief the Path 2 cost between a node and its neighbor
  /// \param s the current node being expanded
  /// \param sp the neighbor node being evaluated
  double path2_cost(SearchNode s, SearchNode sp);

  class HSearch
  {
  public:

    /// \breif default constructor
    HSearch() {};

    /// \breif Initialize the search with a precontructed graph
    HSearch(std::vector<prm::Node>* node_list);

    /// \brief Use default destructor for this and all derived classes
    virtual ~HSearch() = default;

    /// \brief The main routine for the search algorithm
    /// \param s_start the starting node for the path
    /// \param s_goal the goal node for the path
    /// \returns True if a path was found, otherwise False
    virtual bool ComputeShortestPath(const prm::Node & s_start, const prm::Node & s_goal);

    /// \brief All the user to retrive the final path
    /// \returns the final path determined by the search
    std::vector<rigid2d::Vector2D> get_path();

  protected:
    std::vector<prm::Node>* created_graph_p; ///< pointer to a vector of created nodes

    std::vector<SearchNode> open_list; ///< the open list for the current search
    std::vector<SearchNode> closed_list; ///< the closed list for the current search

    std::vector<rigid2d::Vector2D> final_path; ///<assemble the final path based on the goal node

    SearchNode start; ///< the start node for the current search
    rigid2d::Vector2D goal_loc; ///< the start node for the current search

    int id_cnt = 1; ///<tracks # nodes seen during search

    /// \brief a function used to compute the cost for a pair of nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    virtual void ComputeCost(SearchNode &s, SearchNode &sp) = 0;

    /// \brief build the final path based on all of the saved pointers
    /// \param goal the goal node determined by the search
    void assemble_path(SearchNode goal);

    /// \brief calculate the total cost of a node
    /// \param n a node
    /// \returns the total cost
    std::vector<double> f(SearchNode s, SearchNode sp);

    /// \brief calculate the actual path cost
    /// \param s the parent node
    /// \param sp the node to calculate the cost for
    /// \returns the actual path cost from start to sp
    double g(SearchNode s, SearchNode sp);

    /// \brief calculate the estimated cost to goal (heuristic)
    /// \param sp the node to estimate the cost from
    /// \returns the estimated cost
    double h(SearchNode sp);

    /// \brief Compare the current node and its neighbor
    /// \param s the current node
    /// \param s the neioghbor node
    void UpdateVertex(SearchNode s, SearchNode sp);

  };

  class AStar : public HSearch
  {
  public:

    AStar() {};

    /// \brief Initialize the search with the created graph
    /// \param node_list pointer to a vector of Nodes that create a graph
    AStar(std::vector<prm::Node> * node_list) : HSearch(node_list) {};

  protected:
    /// \calculates the path 1 cost between the two nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);
  };

  class ThetaStar : public HSearch
  {

  protected:
    /// \calculates the path 1 or path 2 cost between the two nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);
  };

}

#endif //HSEARCH_INCLUDE_GUARD_HPP
