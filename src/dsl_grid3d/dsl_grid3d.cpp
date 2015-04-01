#include "dsl_grid3d/dsl_grid3d.h"
#include "dsl_grid3d/mesh_utility.h"

namespace dsl_grid3d
{

DslGrid3D::DslGrid3D(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  mesh_filename_("")
{
  if (!nh_private.getParam ("mesh_filename", mesh_filename_))
    mesh_filename_ = "";
  if (!nh_private.getParam ("cells_per_meter", cells_per_meter_))
    cells_per_meter_ = 1.;

  if(mesh_filename_ != "")
  {
    ROS_INFO("Loading occupancy map from mesh file: %s", mesh_filename_.c_str());
    MeshUtility::meshToOccupancyGrid(mesh_filename_, cells_per_meter_, &ogrid_);
  }
  else
  {
    ROS_INFO("Loading blank occupancy map");
    // TODO: Allow user to add lwh and size as parameters, will be overriden if loading mesh
  }

  //Perform dsl gridsearch3D
  gdsl_ = new dsl::GridSearch3D(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getHeight(), ogrid_->getOccupancyMap());
  gdsl_->SetStart(0, 0, 0);
  gdsl_->SetGoal(0, 0, 0);
  //path_marker_pub = nh.advertise<visualization_msgs::Marker>("/map_localize/estimated_path", 1);
  //virtual_image_sub = nh.subscribe<sensor_msgs::Image>("virtual_image", 1, &MapLocalizer::HandleVirtualImage, this, ros::TransportHints().tcpNoDelay());
}

void DslGrid3D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3i pos = ogrid_->positionToGrid(Eigen::Vector3d(msg->x, msg->y, msg->z));
  gdsl_->SetStart(pos(0), pos(1), pos(2));
}
void DslGrid3D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3i pos = ogrid_->positionToGrid(Eigen::Vector3d(msg->x, msg->y, msg->z));
  gdsl_->SetGoal(pos(0), pos(1), pos(2));
}
void DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)
{
  //TODO
}
void DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)
{
  //TODO
}

} // namespace
