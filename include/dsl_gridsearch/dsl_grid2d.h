#ifndef _DSL_GRID_2D_H_
#define _DSL_GRID_2D_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/Mesh.h>
#include <nav_msgs/Path.h>

#include "dsl/travsearch.h"
#include "dsl_gridsearch/occupancy_grid.h"


namespace dsl_gridsearch
{

class DslGrid2D
{
public:
  DslGrid2D(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
  void handleSetStart(const geometry_msgs::PointConstPtr& msg);
  void handleSetGoal(const geometry_msgs::PointConstPtr& msg);
  void handleSetCost(const geometry_msgs::PointConstPtr& msg);
  void handleSetMeshCost(const shape_msgs::MeshConstPtr& msg);

  void publishAllPaths();
  void publishMesh();
  void publishOccupancyGrid();

  void planAllPaths();
  nav_msgs::Path dslPathToRosMsg(const dsl::GridPath& dsl_path);
  bool isPosInBounds(const Eigen::Vector3d& pos);

  dsl::TravSearch* gdsl_;
  dsl::GridPath path_, optpath_;
  OccupancyGrid* ogrid_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher occ_map_viz_pub_;
  ros::Publisher mesh_marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher optpath_pub_;

  ros::Subscriber set_start_sub_;
  ros::Subscriber set_goal_sub_;
  ros::Subscriber set_cost_sub_;
  ros::Subscriber set_mesh_cost_sub_;

  std::string mesh_filename_;
  double cells_per_meter_;
  bool use_textured_mesh_;
  int grid_length_;
  int grid_width_;
};


} // namespace

#endif
