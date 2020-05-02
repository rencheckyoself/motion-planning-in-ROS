#ifndef HSEARCH_INCLUDE_GUARD_HPP
#define HSEARCH_INCLUDE_GUARD_HPP
/// \file
/// \brief A library containing classes to perform various types of search algorithms

#include <memory>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"
#include "roadmap/grid.hpp"

namespace hsearch
{

  /// \brief Used to track if a node is New, on the open, or on the closed list
  enum status {New, Open, Closed};

  /// \brief Information used by a search algorithm
  struct SearchNode
  {
    std::shared_ptr<prm::Node> node_p; ///< contains info about ID, location and, neighbors

    int search_id = 0; ///< a unique id for the search node created

    double f_val; ///< total node cost
    double g_val; ///< path cost from start to this
    double h_val; ///< estimated cost from this to goal

    std::shared_ptr<prm::Node> parent_p; ///< the parent of this node

    status state; ///< current status of the node

    /// \brief Default constructor
    SearchNode() {};

    /// \brief Create a new start node
    /// \param n reference to a graph Node 
    SearchNode(const prm::Node & n);

    /// \brief Custom function used for proper sorting of the open list.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is less than rhs
    bool operator<(const SearchNode &rhs) const;

    /// \brief Custom function used for proper sorting of the open list.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is greater than rhs
    bool operator>(const SearchNode &rhs) const;
  };

  /// \brief Overload the cout operator to print the info in a SearchNode
  /// \param os the output stream
  /// \param n a SearchNode reference
  /// \returns an output stream
  std::ostream & operator<<(std::ostream & os, const SearchNode & n);

  /// \brief The base class to define a heuristic based search algorithm. This class has no ComputeCost funtion which is required to find
  /// the shortest path. This function is defined in the derived class to determine the type of search. Some searched also have a different flow for
  /// finding the shortest path, which is why the ComputeShortestPath method is virtual.
  class HSearch
  {
  public:

    /// \brief default constructor
    HSearch() {};

    /// \brief Initialize the search with a precontructed graph
    /// \param node_list a pointer to a graph
    HSearch(std::vector<prm::Node>* node_list);

    /// \brief Initialize the search with a precontructed graph
    /// \param node_list a pointer to a graph
    /// \param map the known map used to create the graph
    HSearch(std::vector<prm::Node>* node_list, grid::Map map);

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

    grid::Map known_map; ///< Contains all known obstacles and the bounds of the map.

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
    /// \param s the potential parent node
    /// \param sp the node to calculate the cost for
    /// \returns the total cost
    std::vector<double> f(SearchNode s, SearchNode sp);

    /// \brief calculate the actual path cost
    /// \param s the potential parent node
    /// \param sp the node to calculate the cost for
    /// \returns the actual path cost from start to sp
    double g(SearchNode s, SearchNode sp);

    /// \brief calculate the estimated cost to goal (heuristic)
    /// \param sp the node to estimate the cost from
    /// \returns the estimated cost
    double h(SearchNode sp);

    /// \brief Compare the current node and its neighbor
    /// \param s the current node
    /// \param sp the neioghbor node
    void UpdateVertex(SearchNode s, SearchNode sp);

  };

  /// \brief A* Search class derived from the HSearch class
  class AStar : public HSearch
  {
  public:

    /// \brief Initialize the search with the created graph
    /// \param node_list pointer to a vector of Nodes that create a graph
    AStar(std::vector<prm::Node> * node_list) : HSearch(node_list) {};

  protected:
    /// \brief calculates the path 1 cost between the two nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);
  };

  /// \brief Theta* any-angle path planner derived from the HSearch class
  class ThetaStar : public HSearch
  {
  public:

    /// \brief Constructor to initialize a Theta Star Search
    /// \param node_list pointer to a vector of Nodes that create a graph
    /// \param map a known the map used to create the graph
    /// \param buffer a buffer radius to account for in line of sight checks
    ThetaStar(std::vector<prm::Node> * node_list, grid::Map map, double buffer);

  protected:

    double buffer_radius; ///<buffer radius when considering line of sight

    /// \brief calculates the path 1 or path 2 cost between the two nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);
  };

}

#endif //HSEARCH_INCLUDE_GUARD_HPP
