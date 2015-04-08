# DSL Grid3D: D*-lite on a uniformly spaced 3D grid

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



# 2 Quickstart / Minimal Setup

Launch the rviz viewer:

		roslaunch dsl_grid3d rviz.launch

Launch the dsl_grid3d main ros node:

		roslaunch dsl_grid3d dsl_grid3d_campus.launch

Run catkin_make from the root directory.



### 3 Parameters
User must specify either a mesh to load or the size of the occupancy grid.  If both are given, the mesh will be loaded into an occupancy grid of the size given.

* `mesh_filename`: [string] Absolute filepath of a .stl mesh to be loaded into an occupancy grid at runtime.
* `cells_per_meter`: [double] The resolution of the occupancy grid in cells/meter.
* `spline_path_maxvelocity`: [double] Determines the time scaling of the spline interpolated paths.
* `use_textured_mesh`: [bool] Indicates whether a .dae mesh file is availble to be used for visualization.  If mesh_filename is not the empty string, the program will look for a file with the same name with a .dae extension.  This file will be visualized in Rviz.  If set to false, the .stl file will be visualized. 
* `grid_length`: [int] Length of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_width`: [int] Width of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_height`: [int] Height of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_height`: [int] Height of the grid. If less than or equal to 0, the grid will be the same size as the bounding box of the loaded mesh.
* `grid_xmin`: [int] The world frame x value corresponding to the first cell of the grid.
* `grid_ymin`: [int] The world frame y value corresponding to the first cell of the grid.
* `grid_zmin`: [int] The world frame z value corresponding to the first cell of the grid.

