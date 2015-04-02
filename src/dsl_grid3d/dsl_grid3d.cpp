#include "dsl_grid3d/dsl_grid3d.h"
#include "dsl_grid3d/mesh_utility.h"

#include <visualization_msgs/Marker.h>

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
  if (!nh_private.getParam ("spline_path_maxvelocity", spline_path_maxvelocity_))
    spline_path_maxvelocity_ = 1.;
  if (!nh_private.getParam ("use_textured_mesh", use_textured_mesh_))
    use_textured_mesh_ = false;

  if(mesh_filename_ != "")
  {
    ROS_INFO("Loading occupancy map from mesh file: %s", mesh_filename_.c_str());
    MeshUtility::meshToOccupancyGrid(mesh_filename_, cells_per_meter_, &ogrid_);
    ROS_INFO("Loaded succesfully");

    mesh_marker_pub_ = nh.advertise<visualization_msgs::Marker>( "/dsl_grid3d/mesh",  0);
    publishMesh();
  }
  else
  {
    ROS_INFO("Loading blank occupancy map");
    // TODO: Allow user to add lwh and size as parameters, will be overriden if loading mesh
  }

  //Perform dsl gridsearch3D
  ROS_INFO("Building search graph...");
  gdsl_ = new dsl::GridSearch3D(ogrid_->getLength(), ogrid_->getWidth(), ogrid_->getHeight(), ogrid_->getOccupancyMap());
  gdsl_->SetStart(0, 0, 0);
  gdsl_->SetGoal(0, 0, 0);
  ROS_INFO("Graph built");

  occ_map_viz_pub_ = nh_.advertise<visualization_msgs::Marker>( "/dsl_grid3d/occupancy_map",  0);
  path_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/path",  0);
  optpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/optpath",  0);
  splinepath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splinepath",  0);
  splineoptpath_pub_ = nh_.advertise<nav_msgs::Path>( "/dsl_grid3d/splineoptpath",  0);

  publishAllPaths();
  publishOccupancyGrid();

  ROS_INFO("Published occupancy grid");

  set_start_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_start", 10, &DslGrid3D::handleSetStart, this, ros::TransportHints().tcpNoDelay());
  set_goal_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_goal", 10, &DslGrid3D::handleSetGoal, this, ros::TransportHints().tcpNoDelay());
  set_occupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_occupied", 10, &DslGrid3D::handleSetOccupied, this, ros::TransportHints().tcpNoDelay());
  set_unoccupied_sub_ = nh_.subscribe<geometry_msgs::Point>("/dsl_grid3d/set_unoccupied", 10, &DslGrid3D::handleSetUnoccupied, this, ros::TransportHints().tcpNoDelay());

  //TODO: try spinner... 
}

void DslGrid3D::handleSetStart(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3i pos = ogrid_->positionToGrid(Eigen::Vector3d(msg->x, msg->y, msg->z));
  gdsl_->SetStart(pos(0), pos(1), pos(2));

  planAllPaths();
}
void DslGrid3D::handleSetGoal(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3i pos = ogrid_->positionToGrid(Eigen::Vector3d(msg->x, msg->y, msg->z));
  gdsl_->SetGoal(pos(0), pos(1), pos(2));

  planAllPaths();
}
void DslGrid3D::handleSetOccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, true);
  gdsl_->SetCost(gpos(0), gpos(1), gpos(2), DSL_OCCUPIED);

  publishOccupancyGrid();
  planAllPaths();
}
void DslGrid3D::handleSetUnoccupied(const geometry_msgs::PointConstPtr& msg)
{
  Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
  Eigen::Vector3i gpos = ogrid_->positionToGrid(wpos);
  ogrid_->setOccupied(wpos, false);
  gdsl_->SetCost(gpos(0), gpos(1), gpos(2), 0);

  publishOccupancyGrid();
  planAllPaths();
}

void DslGrid3D::planAllPaths()
{
  gdsl_->Plan(path_);
  gdsl_->OptPath(path_, optpath_);
  //gdsl_->SmoothPathOptCost(path_, splinepath_, spline_path_maxvelocity_*cells_per_meter_/10., .4);
  gdsl_->SmoothPathSpline(optpath_, splineoptpath_, spline_path_maxvelocity_*cells_per_meter_/10., .4);
}

void DslGrid3D::publishAllPaths()
{
  path_pub_.publish(dslPathToRosMsg(path_)); 
  optpath_pub_.publish(dslPathToRosMsg(optpath_)); 
  splinepath_pub_.publish(dslPathToRosMsg(splinepath_)); 
  splineoptpath_pub_.publish(dslPathToRosMsg(splineoptpath_)); 
}

nav_msgs::Path DslGrid3D::dslPathToRosMsg(const dsl::GridPath3D &dsl_path)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/world";
  msg.poses.resize(dsl_path.count);
  double xmin = ogrid_->getPmin()(0);
  double ymin = ogrid_->getPmin()(1);
  double zmin = ogrid_->getPmin()(2);
  double offset = 1.0/(2*cells_per_meter_);
  for(int i = 0; i < dsl_path.count; i++)
  {
    msg.poses[i].pose.position.x = dsl_path.pos[3*i]/cells_per_meter_ + offset + xmin;//0.5;
    msg.poses[i].pose.position.y = dsl_path.pos[3*i+1]/cells_per_meter_ + offset + ymin;//0.5;
    msg.poses[i].pose.position.z = dsl_path.pos[3*i+2]/cells_per_meter_ + offset + zmin;//0.5;
  }
  return msg; 
}

void DslGrid3D::publishMesh()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time();
  marker.ns = "dsl_grid3d";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  //only if using a MESH_RESOURCE marker type:
  if(use_textured_mesh_)
  {
    marker.color.a = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_use_embedded_materials = true;

    std::string map_fn(mesh_filename_);
    unsigned int found = map_fn.find_last_of(".");
    std::string texture_fn =  map_fn.substr(0,found) + std::string(".dae");
    marker.mesh_resource = texture_fn;
    std::cout << "Using textured mesh: " << texture_fn << std::endl;
  }
  else
  {
    marker.mesh_resource = std::string(mesh_filename_);
  }

  mesh_marker_pub_.publish( marker );
}

void DslGrid3D::publishOccupancyGrid()
{
  visualization_msgs::Marker occmap_viz;

  std::vector<geometry_msgs::Point> marker_pos;
  int length = ogrid_->getLength();
  int width = ogrid_->getWidth();
  int height = ogrid_->getHeight();
  for(int x = 0; x < length; x++)
  {
    for(int y = 0; y < width; y++)
    {
      for(int z = 0; z < height; z++)
      {
        int idx = x + y*length + z*length*width;
        assert(!(idx >= length*width*height || idx < 0));
        if(ogrid_->getOccupancyMap()[idx] == DSL_OCCUPIED)
        {
          geometry_msgs::Point pt;
          pt.x = x/cells_per_meter_;
          pt.y = y/cells_per_meter_;
          pt.z = z/cells_per_meter_;
          marker_pos.push_back(pt);
        }
      }  
    }
  }
  
  occmap_viz.header.frame_id = "/world";
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl_grid3d";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(0);
  occmap_viz.pose.position.y = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(1);
  occmap_viz.pose.position.z = 1.0/(2.*cells_per_meter_) + ogrid_->getPmin()(2);
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0/cells_per_meter_;
  occmap_viz.scale.y = 1.0/cells_per_meter_;
  occmap_viz.scale.z = 1.0/cells_per_meter_;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;

  occ_map_viz_pub_.publish(occmap_viz);
}

} // namespace
