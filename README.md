# Motion Planning in ROS from Scratch

## Overview

This project is in progress.


  Brief Package Descriptions:
  - `roadmap`: A package with tools to generate various types of graph structured Road Maps. Currently, it supports PRMs and Grids

  Planned additions:
  - Global Planning using Theta*, D* Lite, Potential Fields
  - Local Planning with DWA and MPC

  See the [Full API](https://rencheckyoself.github.io/motion-planning-in-ROS/) for more info.

## How to use

### Probabilistic Road Maps

To generate a PRM launch `roadmap view_prm.launch`. This will create a new PRM and visualize it in Rviz.

Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The graph consists of 500 nodes trying to connect to the 10 nearest neighbors.
![prm_example](roadmap/documentation/prm_example.png)

### Grids

To generate a grid, launch `roadmap view_grid.launch`. This will create a new grid and visualize it in Rviz.

Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The grid has a 5 times finer resolution than the provided map, with black cells as the actual obstacle, gray cells representing cells inside the buffer zone, and white representing the free space.

![grid_example](roadmap/documentation/grid_example.png)

### Heuristic Search on a Known Map (A* and Theta*)

To view the algorithm in action, launch `global_search plan_prm.launch`. This will create a PRM graph, apply A* and Theta* search to it, and visualize the results it in Rviz.

Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.
Change the parameters in `global_search/config/search_params.yaml` to change the start and goal locations.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m. The graph consists of 500 nodes trying to connect to the 10 nearest neighbors.
The green node is the start and the red node is the goal. The black line is the path determined by A* and the orange line is the path determined by Theta*. 

![known_search_example](global_search/documentation/Thetastar_v_Astar.png)

## Background
