#ifndef HSEARCH_INCLUDE_GUARD_HPP
#define HSEARCH_INCLUDE_GUARD_HPP
/// \file
/// \brief A library containing classes to perform various types of search algorithms

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"
#include "roadmap/grid.hpp"

namespace hsearch
{

  /// \brief Used to track if a node is New, on the open, or on the closed list
  enum status {New, Open, Closed};

  /// \brief the key values for a given node
  struct Key
  {
    double k1 = HUGE_VAL; ///< min(g(s), rhs(s)) + h(s, goal) => similar to f cost in non-incremental search
    double k2 = HUGE_VAL; ///< min(g(s), rhs(s)) => tie breaking condition

    /// \brief Custom function used for proper sorting of the open list by comparing the key values.
    /// \param rhs another Key to compare against
    /// \returns True if this Key is less than rhs
    bool operator<(const Key &rhs) const;

    /// \brief Custom function used for proper sorting of the open list by comparing the key values.
    /// \param rhs another Key to compare against
    /// \returns True if this Key is less than rhs
    bool operator>(const Key &rhs) const;
  };

  /// \brief Information used by a search algorithm
  struct SearchNode
  {
    std::shared_ptr<prm::Node> node_p = nullptr; ///< contains info about ID, location and, neighbors

    int search_id = 0; ///< a unique id for the search node created

    double f_val = HUGE_VAL; ///< total node cost
    double g_val = HUGE_VAL; ///< path cost from start to this
    double h_val = HUGE_VAL; ///< estimated cost from this to goal

    double rhs_val = HUGE_VAL; ///< another estimate of the start distance used for incremental search methods

    Key key_val;

    std::shared_ptr<prm::Node> parent_p = nullptr; ///< the parent of this node

    status state = New;; ///< current status of the node

    /// \brief Default constructor
    SearchNode() {};

    /// \brief Create a new start node
    /// \param n reference to a graph Node
    SearchNode(const prm::Node & n);

    /// \brief Update the key values for a node
    /// \param km (optional) the key modifier used in an incremental D* search, defaults to 0
    void CalcKey(double km = 0);

    /// \brief Custom function used for proper sorting of the open list by comparing the key values.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is less than rhs
    bool operator<(const SearchNode &rhs) const;

    /// \brief Custom function used for proper sorting of the open list by comparing the key values.
    /// \param rhs another SearchNode to compare against
    /// \returns True if this node is greater than rhs
    bool operator>(const SearchNode &rhs) const;
  };

  /// \brief Overload the cout operator to print the info in a SearchNode
  /// \param os the output stream
  /// \param n a SearchNode reference
  /// \returns an output stream
  std::ostream & operator<<(std::ostream & os, const SearchNode & n);

  /// \brief Overload the cout operator to print the info in a Key
  /// \param os the output stream
  /// \param n a Key reference
  /// \returns an output stream
  std::ostream & operator<<(std::ostream & os, const Key & k);

  /// \brief The base class to define a heuristic based search algorithm. This class has no ComputeCost funtion which is required to find
  /// the shortest path. This function is defined in the derived class to determine the type of search. Some searched also have a different flow for
  /// finding the shortest path, which is why the ComputeShortestPath method is virtual. Currently this class will only plan against a precontructed graph.
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
    // HSearch(std::vector<prm::Node>* node_list, grid::Map map);

    /// \brief Initialize the search with a precontructed graph
    /// \param node_list a pointer to a graph
    /// \param map the known map used to create the graph
    HSearch(grid::Map map);

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

    /// \brief Get all of the nodes expanded during the search
    /// \returns a vector of points that were expanded
    std::vector<rigid2d::Vector2D> get_expanded_nodes();

  protected:
    std::vector<prm::Node>* created_graph_p; ///< pointer to a vector of created nodes

    std::vector<SearchNode> open_list; ///< the open list for the current search
    std::vector<SearchNode> closed_list; ///< the closed list for the current search

    std::vector<rigid2d::Vector2D> final_path; ///<assemble the final path based on the goal node

    std::vector<rigid2d::Vector2D> expanded_nodes; ///< a list of points that were expanded (popped off the open list) during the most recent search

    SearchNode start; ///< the start node for the current search
    rigid2d::Vector2D goal_loc; ///< the goal node for the current search

    int id_cnt = 1; ///<tracks # nodes seen during search

    /// \brief a function used to compute the cost for a pair of nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    virtual void ComputeCost(SearchNode &s, SearchNode &sp) = 0;

    /// \brief build the final path based on all of the saved pointers
    /// \param goal the goal node determined by the search
    virtual void assemble_path(SearchNode goal);

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

    grid::Map known_map; ///< Contains all known obstacles and the bounds of the map.

    double buffer_radius; ///<buffer radius when considering line of sight

    /// \brief calculates the path 1 or path 2 cost between the two nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);
  };

  /// \brief a generic class to perform LPA* search
  class LPAStar : public HSearch
  {
  public:

    /// \brief Default Constructor
    LPAStar();

    /// \brief provide the search with the beginning state of the map
    /// \param grid_graph reference to an existing grid
    /// \param start_loc the location of the starting point in integer coordinates on the provided grid
    /// \param goal_loc the location of the goal point in integer coordinates on the provided grid
    LPAStar(std::vector<std::vector<prm::Node>>* grid_graph, grid::Grid* base_grid, rigid2d::Vector2D start_loc, rigid2d::Vector2D goal_loc);

    /// \brief The main loop for to find the shortest path
    /// \returns True if a path was found, otherwise False
    bool ComputeShortestPath();

    /// \brief Take in simulated sensor information and determine if there is a change in the map
    /// \param points pairs of grid cell locations and new occupancy data to potentially update.
    /// \returns True if the information in points actually caused a change in the occupancy data from free to occupied, otherwise False.
    bool MapChange(std::vector<std::pair<rigid2d::Vector2D, signed char>> points);

  protected:

    std::vector<std::vector<prm::Node>>* created_graph_p; ///< pointer to a 2D vector of created nodes

    grid::Grid* known_grid_p; ///<pointer to the known grid containing current occupancy data

    std::unordered_map<int, SearchNode> standby; ///< all nodes not on the open list

    int start_id; ///< ID of the SearchNode conttaining the start of the search
    int goal_id; ///< ID of the SearchNode conttaining the goal of the search

    /// \brief build the final path based on all of the saved pointers
    /// \param goal the goal SearchNode
    void assemble_path(SearchNode goal);

    /// \brief Update a node
    /// \param u the id of a node to update
    void UpdateVertex(int u);

    /// \brief a function used to compute the cost for a pair of nodes
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    void ComputeCost(SearchNode &s, SearchNode &sp);

    /// \brief a function used calculate traversal cost between 2 nodes based on the known_map.
    /// Uses the stored point in each node to determine if a cell is free or occupied. If one is occupied the cost
    /// is set to HUGE_VAL, otherwise use the straight line distance.
    /// \param s the current node being expanded
    /// \param sp the neighbor node being evaluated
    /// \returns the cost to traverse from sp to s
    double edge_cost(SearchNode &s, SearchNode &sp);

    /// \brief Locate a node in the open list or standby
    /// \param u_id the id of a node to find
    /// \returns a pointer to the node
    SearchNode* locate_node(int u_id);

    /// \brief Get a reference to the goal node
    /// \returns goal key
    Key get_goal_key();

    /// \brief Check the local consistency of a node
    /// \returns true if the goal is locally consistent, otherwise false
    bool goal_is_consistent();

    /// \brief Determine if a node is locally is consistent
    /// \param u a node to evaluate
    /// \returns True if the node is locally consistent
    bool is_consistent(SearchNode u) const;
  };
}

#endif //HSEARCH_INCLUDE_GUARD_HPP
