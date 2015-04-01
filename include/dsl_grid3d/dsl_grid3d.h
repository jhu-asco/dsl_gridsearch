#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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

  void publishPath();
  void publishOptPath();
  void publishMesh();
  void publishOccupancyGrid();

  dsl::GridSearch3D* gdsl_;
  OccupancyGrid* ogrid_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string mesh_filename_;
  double cells_per_meter_;
};


} // namespace

#endif
