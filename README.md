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

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The graph consists of 500 nodes trying to connect to the 10 nearest neighbors.

<img src="roadmap/documentation/prm_example.png" width="500">

### Grids

To generate a grid, launch `roadmap view_grid.launch`. This will create a new grid and visualize it in Rviz.

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The grid has a 5 times finer resolution than the provided map, with black cells as the actual obstacle, gray cells representing cells inside the buffer zone, and white representing the free space.

<img src="roadmap/documentation/grid_example.png" width="500">

### Heuristic Search on a Known Map (A* and Theta*)

To view the algorithm in action, launch `global_search plan_prm.launch`. This will create a PRM graph, apply A* and Theta* search to it, and visualize the results it in Rviz.

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.
- Change the parameters in `global_search/config/search_params.yaml` to change the start and goal locations.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m. The graph consists of 500 nodes trying to connect to the 10 nearest neighbors.
The green node is the start and the red node is the goal. The black line is the path determined by A* and the orange line is the path determined by Theta*.

<img src="global_search/documentation/Thetastar_v_Astar.png" width="500">

### Iterative Search on an Unknown Map (LPA* and D* Lite)

To view the LPA* algorithm, launch `global_search lpastar_grid.launch`. This will create 2 grids, one using the stored obstacle data and one only accounting for the map boundary. LPA* is provided the empty grid and will plan an initial path between the start and goal locations. The known grid will be used to simulate a camera or some other sensor detecting a change in the environment, which will trigger LPA* to replan given the new information.

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.
- Change the parameters in `global_search/config/search_params.yaml` to change the start and goal locations.

The following gif was taken using a cell size of 0.2m with a buffer radius of 0.15m and a grid resolution of 1.
The green node is the start and the red node is the goal, with the black line showing the final path determined by LPA* for the current map data. The faded area of the map is assumed by the search to be completely free and occupancy data is filled in one row at a time from the bottom up. Cells marked with a light blue square indicate that it was updated during the most recent search.

<img src="global_search/documentation/lpastar.gif" width="500">

To view the D* Lite algorithm, launch `global_search dstarlite_grid.launch`. This will create 2 grids, one using the stored obstacle data and one only accounting for the map boundary. D* Lite is provided the empty grid and will plan an initial path between the start and goal locations. The known grid will be used to simulate a sensor mounted to the robot to detect a change in the environment within a given radius around the robot. This will trigger D* Lite to replan given the new information.

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.
- Change the parameters in `global_search/config/search_params.yaml` to change the start and goal locations and the sensor range.

The following gif was taken using a cell size of 0.2m with a buffer radius of 0.15m, a grid resolution of 1, and a simulated sensor range of 0.6m.
The green node is the start and the red node is the goal and the robot is the blue cube. The black line represents the path the robot has taken and the orange line is the path determined by D* Lite for the current map data. The faded area of the map is assumed by the search to be completely free and occupancy data is filled as the simulated sensor is able to detect the cell. Cells marked with a light blue square indicate that it was updated during the most recent search.

<img src="global_search/documentation/dstarlitev2.gif" width="500">

### Potential Fields

To view the algorithm in action, launch `global_search plan_potential_fields.launch`. This will use the existing map data to plan a path from start to goal using the standard potential field algorithm. This implementation does not currently  provide a means of escaping local minima and assumes a fully known map.

- Change the parameters in `roadmap/config/map_params.yaml` to customize the components of the map.
- Change the parameters in `global_search/config/search_params.yaml` to change the start and goal locations and potential field parameters.

The following gif was taken using a cell size of 0.2 with the following potential field parameters:
  ```
  att_weight: 0.6 # weighting factor the attactive component
  dgstar: 3 # piecewise threshold for attractive gradient
  rep_weight: 0.1 # weighting factor the repulsive component
  Qstar: 0.4 # obstacle range of influence
  epsilon: 0.05 # termination threshold
  zeta: 0.01 # step size
  ```
The green node is the start and the red node is the goal and the orange line is the path determined by the potential field algorithm.

<img src="global_search/documentation/potfield.gif" width="500">

## Background
