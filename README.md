# Motion Planning in ROS from Scratch

This project is in progress.

Currently implemented:

- Probabilistic Road Map creation: 
  How to run: launch `view_world.launch`
  
  Description:
  - `prm.cpp`: Library to build the PRM
  - `draw_world.cpp`: Draw the map/obstacles in RViz.
  - `make_raodmap.cpp`: Create the PRM and visualize in RViz

Planned additions:
- Build a Grid for the map
- Global Planning using A*, Incremental Phi*, Potential Fields
- Local Planning with DWA and MPC
