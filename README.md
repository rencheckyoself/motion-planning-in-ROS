# Motion Planning in ROS from Scratch

## Overview

This project is in progress.


  Brief Package Descriptions:
  - `roadmap`: A package with tools to generate various types of graph structured Road Maps. Currently, it supports PRMs and Grids

Planned additions:
- Global Planning using Theta*, D* Lite, Potential Fields
- Local Planning with DWA and MPC


## How to use:

### Probabilistic Road Maps

To generate a PRM launch `view_prm.launch`. This will create a new PRM and visualize it in Rviz.

Change the parameters in `config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The graph consists of 500 nodes trying to connect to the 10 nearest neighbors.
![prm_example](roadmap/documentation/prm_example.png)

### Grids

To generate a grid, launch `view_grid.launch`. This will create a new grid and visualize it in Rviz.

Change the parameters in `config/map_params.yaml` to customize the components of the map.

The following image was taken using a cell size of 0.2m with a buffer radius of 0.15m.
The grid has a 5 times finer resolution than the provided map, with black cells as the actual obstacle, gray cells representing cells inside the buffer zone, and white representing the free space.

![grid_example](roadmap/documentation/grid_example.png)
