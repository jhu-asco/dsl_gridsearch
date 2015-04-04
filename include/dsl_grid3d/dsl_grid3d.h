#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/Mesh.h>
#include <nav_msgs/Path.h>

#include "dsl/gridsearch3d.h"
#include "dsl_grid3d/occupancy_grid.h"


namespace dsl_grid3d
{

class DslGrid3D
{
public:
  DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
  void handleSetStart(const geometry_msgs::PointConstPtr& msg);
  void handleSetGoal(const geometry_msgs::PointConstPtr& msg);
  void handleSetOccupied(const geometry_msgs::PointConstPtr& msg);
  void handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg);
  void handleAddMesh(const shape_msgs::MeshConstPtr& msg);
  void spin(const ros::TimerEvent& e);

  void publishAllPaths();
  void publishMesh();
  void publishOccupancyGrid();

  void planAllPaths();
  nav_msgs::Path dslPathToRosMsg(const dsl::GridPath3D& dsl_path);
  bool isPosInBounds(const Eigen::Vector3d& pos);

  dsl::GridSearch3D* gdsl_;
  dsl::GridPath3D path_, optpath_;
  dsl::GridPath3DPlusTime splinepath_, splineoptpath_;
  OccupancyGrid* ogrid_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher occ_map_viz_pub_;
  ros::Publisher mesh_marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher optpath_pub_;
  ros::Publisher splinepath_pub_;
  ros::Publisher splineoptpath_pub_;

  ros::Subscriber set_start_sub_;
  ros::Subscriber set_goal_sub_;
  ros::Subscriber set_occupied_sub_;
  ros::Subscriber set_unoccupied_sub_;

  ros::Timer timer;

  std::string mesh_filename_;
  double cells_per_meter_;
  double spline_path_maxvelocity_;
  bool use_textured_mesh_;
  int grid_length_;
  int grid_width_;
  int grid_height_;
};


} // namespace

#endif
