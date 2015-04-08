# DSL Grid3D: D*-lite on a uniformly spaced 3D grid

This is an implementation of D*-lite graph search on a uniformly spaced 3D grid for use in global path planning.  This package provides the ability to create an occupancy grid from a .stl mesh or to specify a grid of a given size.  The user can specify start and goal positions by publishing to the relevant topics, and the generated paths will be published by the node.  The user can also publish messages to set grid cells to be occupied or unoccupied, or mesh messages can be sent to set all cells which intersect with the mesh as occupied.

# 1 Installation
We tested DSL Grid3D on Ubuntu 12.04 (Precise) and ROS hydro.

In your ROS package path, clone the repository:

    git clone https://github.com/repo.git

Install the included dsl library

    cd repo/extern/dsl
    mkdir build
    cd build
    cmake ..
    sudo make install

Run catkin_make from the root directory, as usual.


# 2 Quickstart / Minimal Setup

Launch the rviz viewer:

		roslaunch dsl_grid3d rviz.launch

Launch the dsl_grid3d main ros node and load a mesh:

		roslaunch dsl_grid3d dsl_grid3d_campus.launch

# 3 Topics
## 3.1 Subscribed
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the start position.
* `/dsl_grid3d/set_start`: [geometry_msgs::Point] Used to set the goal position.
* `/dsl_grid3d/set_occupied`: [geometry_msgs::Point] Used to set a position as occupied.
* `/dsl_grid3d/set_unoccupied`: [geometry_msgs::Point] Used to set a position as unoccupied.
* `/dsl_grid3d/set_mesh_occupied`: [shape_msgs::Mesh] Used to set cells intersecting with a mesh as occupied.

## 3.2 Published 
* `/dsl_grid3d/occupancy_map`: [visualization_msgs::Marker] A marker for the occupancy grid to be displayed in Rviz.
* `/dsl_grid3d/mesh`: [visualization_msgs::Marker] A marker for the mesh to be displayed in Rviz.
* `/dsl_grid3d/path`: [nav_msgs::Path] The generated path from start to goal.
* `/dsl_grid3d/optpath`: [nav_msgs::Path] An optimized version of the path which removes unnecessary waypoints.
* `/dsl_grid3d/splinepath`: [nav_msgs::Path] A spline interpolation of the waypoints from the original path.
* `/dsl_grid3d/splineoptpath`: [nav_msgs::Path] A spline interpolation of the waypoints from the optimized path.


# 4 Parameters
The user must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `mesh_filename`: [string] Absolute filepath of a .stl mesh to be loaded into an occupancy grid at runtime.
* `cells_per_meter`: [double] The resolution of the occupancy grid in cells/meter.
* `spline_path_maxvelocity`: [double] Determines the time scaling of the spline interpolated paths.
* `use_textured_mesh`: [bool] Indicates whether a .dae mesh file is availble to be used for visualization.  If mesh_filename is not the empty string, the program will look for a file with the same name with a .dae extension.  This file will be visualized in Rviz.  If set to false, the .stl file will be visualized. 
* `grid_length`: [int] Length of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_width`: [int] Width of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_height`: [int] Height of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_xmin`: [int] The world frame x value corresponding to the first cell of the grid.
* `grid_ymin`: [int] The world frame y value corresponding to the first cell of the grid.
* `grid_zmin`: [int] The world frame z value corresponding to the first cell of the grid.

