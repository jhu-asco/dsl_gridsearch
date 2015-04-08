#ifndef _OCCUPANCY_GRID_H_
#define _OCCUPANCY_GRID_H_

#include <Eigen/Core>
#include <Eigen/Dense>

#define DSL_OCCUPIED 1e10


namespace dsl_grid3d
{

class OccupancyGrid
{
public:
  OccupancyGrid(double* occupancy_map, int length, int width, int height, Eigen::Vector3d pmin, Eigen::Vector3d pmax, double scale);
  OccupancyGrid(int length, int width, int height, Eigen::Vector3d pmin, Eigen::Vector3d pmax, double scale);

  void mergeGrid(OccupancyGrid* ogrid);
  bool isOccupied(const Eigen::Vector3d& p);
  bool isOccupied(const Eigen::Vector3i& gp);
  bool isInGrid(const Eigen::Vector3i& gp); 
  void setOccupied(const Eigen::Vector3d& p, bool set);
  int positionToIndex(const Eigen::Vector3d& p);
  Eigen::Vector3i positionToGrid(const Eigen::Vector3d& p);
  Eigen::Vector3d gridToPosition(const Eigen::Vector3i& gp);

  Eigen::Vector3d getPmin();
  Eigen::Vector3d getPmax();
  double getScale();
  int getLength();
  int getWidth();
  int getHeight();
  double* getOccupancyMap();

private:
  double* occupancy_map_;
  int length_;
  int width_;
  int height_; 
  double scale_;
  Eigen::Vector3d pmin_;
  Eigen::Vector3d pmax_;


};

} // namespace

#endif
