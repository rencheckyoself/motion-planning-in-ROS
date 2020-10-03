# Motion Planning in ROS from Scratch

## Overview

  Brief Package Descriptions:
  - `roadmap`: A package with tools to generate various types of graph structured Road Maps. Currently, it supports PRMs and Grids
  - `global_search`: A package with various methods to perform global search on the different maps in the road map package
  - `mppi_control`: A package to perform MPPI control for waypoint following


  See my [portfolio](https://mrencheck.wixsite.com/michaelrencheck/rosmotionplanningfromscratch) for a summary video.

  Future additions:
  - A local planner package to experiment with Dynamic Window Approach.

## How to use

Clone the repository and then use the included .rosinstall file to also download the other required packages.

Source then build allow of the downloaded packages, then see below how to use each package.   

See the [Full API](https://rencheckyoself.github.io/motion-planning-in-ROS/) for more info on how to use the various libraries.

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

To view the algorithm in action, launch `global_search plan_potential_fields.launch`. This will use the existing map data to plan a path from start to goal using the standard potential field algorithm. This implementation does not currently provide a means of escaping local minima and assumes a fully known map.

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


### MPPI

To view the algorithm in action, launch `mppi_control turtlebot_mppi.launch`. After launching, call the `/start` service from the terminal to begin the waypoint following. This will apply the mppi control algorithm to calculate a control sequence to drive the robot to a series of waypoints. This package depends on a couple of packages located in my other `ros_navigation_from_scratch` repo. Use the included .rosinstall file to ensure you get the correct packages.

- Change parameters in the mppi_control/config/control_param.yaml to tune the controller

This example is using MPPI control to select wheel velocities for a differential drive robot to drive through consecutive waypoints. Each waypoint is an (x,y,heading) tuple. The parameters used are shown in the configuration file. Due to the random sampling each run is slightly different, but exhibit the same general behavior.

<img src="mppi_control/documentation/ROS_mppi_waypoints.gif" width="500">

Also included in `mppi_control/testing_files` is a python-only script to perform the same algorithm. To use this script, execute the `mppi.py` file. It is currently configured to have a unicycle model robot follow waypoints. Below are some results for various robots and tasks:

The first plot is using the unicycle kinematic model to solve the parallel parking problem. The output of the control algorithm is linear and angular velocities. See the python script for all of the parameters.

<img src="mppi_control/testing_files/unicycle.gif" width="500">

The plot below is using the differential drive kinematic model to solve the parallel parking problem. The output of the control algorithm is right and left wheel velocities. See the python script for all of the parameters.

<img src="mppi_control/testing_files/diff_drive.gif" width="500">

The third plot is using the unicycle kinematic model to follow a series of waypoint. The output of the control algorithm is linear and angular velocities. See the python script for all of the parameters.

<img src="mppi_control/testing_files/waypoints-unicycle.gif" width="500">

## A Brief Background

### Probabilistic Road Map

A PRM is a means to efficiently constructing a system of valid pathways through an environment as it has the advantage to plan in high dimensional configuration spaces. The assembly starts by randomly sampling states and only keeping them if they are a valid. In this implementation, a valid node is an x,y position that is not within the bounds of an obstacle or its buffer zone. After N number of valid nodes have been sampled each node is connected to it's k-nearest neighbors along valid straight line paths. In this implementation a path or edge is considered valid if it does not intersect an obstacle or it's buffer zone.

The challenging part of implementing a PRM is identifying how to determine if a node/edge is valid. This implementation currently only supports convex obstacles and expects that the vertices are provided in counterclockwise order. The collision detection is as follows:
- To determine if a sampled node is inside of an obstacle, test if the state is on the same side of all of the line segments.
- To determine if a sampled node is inside the buffer zone, calculate that shortest distance to each line segment and compare it to the desired buffer distance.
- To determine if an edge between two nodes is

### Grids

Another method of descretizing a continuous environment is told build an occupancy grid based on the known information. For this implementation everything inside an obstacle or within a certain distance away from the obstacle is classified as impassable. This can then be used to create a cost map of 0 for the unoccupied cells or infinity for the impassable ones. However, depending on the application, the occupancy grid data can be used to create a non-binary cost map to more accurately reflect the environment.

The benefit of using a grid is that it is able to easily break up an known area, this descretization may cause a loss in map features if the resolution is too low (the cells are too large). The resolution can always be increased to capture finer details, but at the cost of more computational time required.

### A*/Theta* Search
The A* search algorithm is the genesis of most of the heuristic based search methods. It is commonly used because it is complete (a solution will be found, if one exists) and it is optimal (given the heuristic). A* fits under the best-first search category as at each step in the process the search algorithm will always choose to step toward the best available based on the cost function and the heuristic. When applying A* search to navigation, each increment of the search is limited to evaluating the connected neighbors of a a node and, as a result, the path generated often jagged and requires smoothing to obtain a usable/driveable path; this is the problem Theta* solves. Theta* is classified as an Any-Anlge planner and follows a nearly identical structure as A*, but is not constrained by only connecting immediate neighbors. The Theta* algorithm relies on determining line of sight between two points and using this information the algorithm can connect more than just immediate neighbors.

This search was used on the PRM representation, but could also be applied to the grid.

### LPA*/D* Lite
Lifelong Planning A* and D* Lite are iterative heuristic based search methods and are design to efficiently replan for a changing/unknown environment. LPA* will always maintain the optimal path between a given start and end point, and for the first iteration will perform very similarly to an A* search. However, once a change in the map is detected, the LPA* algorithm is able to used the results of the previous search to replan without fully starting from scratch. D* Lite is an extension of LPA* to adapt the algorithm to a moving robot.

D* Lite will always maintain the optimal path between the goal and the robots current location. As the robot travels, its sensors provide information to update the known map. Similarly to LPA*, it leverages previous information to efficiently replan until a new path is found.

Because these algorithms require a map that can be updated, these algorithms were applied using the grid.

### Potential Fields
The potential fields planning method is powerful because it supports planning in a continuous environment, so no need to use a PRM or grid to traverse from the start to the finish. This method simulates "magnetic" forces that act on the robot in order to traverse the environment. The goal location acts as an attractive force, always pulling the robot towards it. Then each obstacle acts as a repulsive force that pushes the robot away if it comes to close to the obstacle. These "forces" are used to generate a velocity vector for how the robot should move. For sparse environments, the simple version of potential fields works well after tuning the parameters. However if there is a dense region of obstacles the repulsive forces can cause the robot to become stuck in a local minimum and extra logic needs to be implemented for escaping.

### MPPI Control
This control algorithm is an iterative approach to determining a sequence of commands that allow the robot to complete a task as defined by a provided cost function. It accomplishes this by planning for a sequence of controls then randomly and proportionally perturbing the sequence based on the quality of the perturbation. After the perturbations are incorporated the robot is issued the first command in the control sequence and the process is repeated. Like other optimal control algorithms, this one also requires a well formed cost function to define the desired behavior as well as properly tuning the parameters.  

## References and Resources

- LaValle, Steven M. Planning algorithms. Cambridge University press, 2006.

- Choset, Howie M., et al. Principles of robot motion: theory, algorithms, and implementation. MIT press, 2005.

- Latombe, Lydia E. Kavraki Jean-Claude. ”Probabilistic Roadmaps for Robot Path Planning.” Practical motion planning in robotics: current aproaches and future challenges (1998): 33-53.

- Daniel, Kenny, et al. ”Theta*: Any-angle path planning on grids.” Journal of Artificial Intelligence Research 39 (2010): 533-579.

- Koenig, Sven, and Maxim Likhachev. ”Fast replanning for navigation in unknown terrain.” IEEE Transactions on Robotics 21.3 (2005): 354-363.

- Williams, Grady, Andrew Aldrich, and Evangelos Theodorou. "Model predictive path integral control using covariance variable importance sampling." arXiv preprint arXiv:1509.01149 (2015).

- Abraham, Ian, et al. "Model-Based Generalization Under Parameter Uncertainty Using Path Integral Control." IEEE Robotics and Automation Letters 5.2 (2020): 2864-2871.
