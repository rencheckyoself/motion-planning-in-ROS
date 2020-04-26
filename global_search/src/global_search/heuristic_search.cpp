/// \file
/// \brief A library containing classes to perform various types of search algorithms

#include <memory>
#include <cmath>
#include "rigid2d/rigid2d.hpp"
#include "roadmap/prm.hpp"

namespace hsearch
{

  SearchNode::SearchNode(prm::Node *n)
  {
    node_p = n;
    f_val = HUGE_VAL;
    g_val = HUGE_VAL;
    h_val = HUGE_VAL;

    parent_p = this;

    state = New;
  }

  HSearch::ComputeShortestPath(prm::Node* s_start, prm::Node* s_goal)
  {
    // Initialize the start node
    start = SearchNode(s_start);
    start.status = Open;
    start.g_val = 0;
    start.h_val = h(start);
    start.f_val = 0;

    start.parent = &start

    // Initialize the goal node

    open_list.push_back(start);
    std::make_heap(open_list.begin(), open_list.end());

    while(open_list.size() != 0)
    {
      // Get the node with the minimum total cost
      std::head_pop(open_list.begin(), open_list.end());
      auto cur_s = open_list.back();
      open_list.pop_back();

      cur_s.status = Closed;

      // check if this is the goal
      if (cur_s == goal)
      {

      }

    }


  }

  AStar::AStar(std::vector<prm::Node>* node_list)
  {
    created_graph_p = node_list;
  }

}
