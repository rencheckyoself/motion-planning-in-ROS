/// \file
/// \brief A library containing classes to perform various types of search algorithms

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include "global_search/heuristic_search.hpp"
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"

namespace hsearch
{

  SearchNode::SearchNode(const prm::Node & n)
  {
    node_p = std::make_shared<prm::Node>(n);
    f_val = HUGE_VAL;
    g_val = HUGE_VAL;
    h_val = HUGE_VAL;

    parent_p = nullptr;

    state = New;
  }

  bool SearchNode::operator>(const SearchNode &rhs) const
  {
    if(f_val == rhs.f_val) return h_val > rhs.h_val;
    else return f_val > rhs.f_val;
  }

  bool SearchNode::operator<(const SearchNode &rhs) const
  {
    if(f_val == rhs.f_val) return h_val > rhs.h_val;
    else return f_val > rhs.f_val;
  }

  std::ostream & operator<<(std::ostream & os, const SearchNode & n)
  {
    os << "Node ID: " << n.node_p->id << "\n\t" << "Cur Cost: " << n.f_val << "\n\t" << "Parent ID: " << n.parent_p->id << std::endl;
    return os;
  }

  HSearch::HSearch(std::vector<prm::Node>* node_list)
  {
    created_graph_p = node_list;
  }

  bool HSearch::ComputeShortestPath(const prm::Node & s_start, const prm::Node & s_goal)
  {

    goal_loc = s_goal.point;

    // Initialize the start node
    start = SearchNode(s_start);

    start.search_id = 0;

    start.state = Open;
    start.g_val = 0;
    start.h_val = h(start);
    start.f_val = 0;

    start.parent_p = nullptr;

    open_list.push_back(start);
    std::make_heap(open_list.begin(), open_list.end(), std::greater<>{});

    while(open_list.size() != 0)
    {
      // std::cout << "open_list size: " << open_list.size() << std::endl;
      // Get the node with the minimum total cost
      std::pop_heap(open_list.begin(), open_list.end(), std::greater<>{});
      auto cur_s = open_list.back();
      open_list.pop_back();

      // std::cout << "Current Node Cost: " << cur_s.f_val << std::endl;
      // std::cout << "End Node Cost: " << open_list.back().f_val << std::endl;

      // std::cout << "Current Node: " << cur_s.node_p->point << "\n";

      // check if cur_s is the goal
      if (cur_s.node_p->point == goal_loc)
      {
          assemble_path(cur_s);
          return true;
      }

      // Add current node to the closed list
      closed_list.push_back(cur_s);
      cur_s.state = Closed;

      // Expand the search to the neighbors of the current node
      for(auto node_id : cur_s.node_p->id_set)
      {

        // std::cout << "s' ID: " << node_id << "\n";

        auto result = std::find_if(closed_list.begin(), closed_list.end(), [node_id](SearchNode n) {return n.node_p->id == node_id;});

        // std::cout << "On closed list? " << result - closed_list.begin() << "\n";

        // Check if the node is not on the closed list and try to update/create it
        if(result == closed_list.end())
        {

          result = std::find_if(open_list.begin(), open_list.end(), [node_id](SearchNode n) {return n.node_p->id == node_id;});

          // std::cout << "On open list? " << result - open_list.begin() << "\n";

          SearchNode neighbor;

          // check if the node is not on the open list and create it
          if(result == open_list.end())
          {
            SearchNode buf(created_graph_p->at(node_id));
            buf.search_id = id_cnt;
            id_cnt++;
            neighbor = buf;
          }
          else // otherwise use the
          {
            neighbor = *result;
          }

          // calculate the cost and update the cost/parent if needed
          ComputeCost(cur_s, neighbor);

          if (neighbor.state == New) // add node to the heap
          {
            neighbor.state = Open;
            open_list.push_back(neighbor);
          }
          else // update the node already in the heap
          {
            open_list.at(std::distance(open_list.begin(),result)) = neighbor;
          }

          // std::cout << neighbor;

          // Update the heap order
          push_heap(open_list.begin(), open_list.end(), std::greater<>{});
        }
      }
    }
    return false;
  }

  void HSearch::assemble_path(SearchNode goal)
  {
    // add the goal to the path
    final_path.push_back(goal.node_p->point);

    auto cur_node = goal;

    // follow the parent points back to the starting node and store each location
    while (cur_node.parent_p != nullptr)
    {
      final_path.push_back(cur_node.parent_p->point);

      auto next_id = cur_node.parent_p->id;

      // search for the parent on the closed list
      auto result = std::find_if(closed_list.begin(), closed_list.end(), [next_id](SearchNode n) {return n.node_p->id == next_id;});

      if(result != closed_list.end())
      {
        cur_node = *result;
      }
      else
      {
        // search for the parent on the open list
        result = std::find_if(open_list.begin(), open_list.end(), [next_id](SearchNode n) {return n.node_p->id == next_id;});
        cur_node = *result;
      }
    }
  }

  std::vector<rigid2d::Vector2D> HSearch::get_path()
  {
    return final_path;
  }

  std::vector<double> HSearch::f(SearchNode s, SearchNode sp)
  {
    double buf_h = h(sp);
    double buf_g = g(s, sp);
    double buf_f = buf_g + buf_h;

    return {buf_f, buf_g, buf_h};
  }

  double HSearch::g(SearchNode s, SearchNode sp)
  {
    return (s.g_val + sp.node_p->point.distance(s.node_p->point));
  }

  double HSearch::h(SearchNode sp)
  {
    rigid2d::Vector2D pt1 = sp.node_p->point;

    return pt1.distance(goal_loc);
  }

  void AStar::ComputeCost(SearchNode &s, SearchNode &sp)
  {
    auto cost = f(s, sp);
    // If the path from s to s' is cheaper than the existing one, update it.
    if(cost.at(0) < sp.f_val)
    {
      sp.f_val = cost.at(0);
      sp.g_val = cost.at(1);
      sp.h_val = cost.at(2);

      sp.parent_p = s.node_p;
    }
  }
}
