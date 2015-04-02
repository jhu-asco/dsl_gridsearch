#include "dsl_grid3d/occupancy_grid.h"

using namespace Eigen;

namespace dsl_grid3d
{
 
OccupancyGrid::OccupancyGrid(double* occupancy_map, int length, int width, int height, Vector3d pmin, Vector3d pmax, double scale) :
  occupancy_map_(occupancy_map),
  length_(length),
  width_(width),
  height_(height),
  scale_(scale),
  pmin_(pmin),
  pmax_(pmax)
{

}

bool OccupancyGrid::isOccupied(const Vector3d& p)
{
  int idx = positionToIndex(p);
  assert((idx >= 0) && (idx < length_*width_*height_));

  return occupancy_map_[idx] > 0;
}

void OccupancyGrid::setOccupied(const Vector3d& p, bool set)
{
  int idx = positionToIndex(p);
  assert((idx >= 0) && (idx < length_*width_*height_));

  if(set)
    occupancy_map_[idx] = DSL_OCCUPIED;
  else
    occupancy_map_[idx] = 0;
}

int OccupancyGrid::positionToIndex(const Vector3d& p)
{
  Vector3d offset = (p-pmin_)*scale_;
  int x = (int)floor(offset(0));
  int y = (int)floor(offset(1));
  int z = (int)floor(offset(2));
  return x + y*length_ + z*length_*width_;
}

Vector3i OccupancyGrid::positionToGrid(const Vector3d& p)
{
  Vector3d offset = (p-pmin_)*scale_;
  Vector3i g((int)floor(offset(0)), (int)floor(offset(1)),(int)floor(offset(2)));
  return g;
}

int OccupancyGrid::getLength()
{
  return length_;
}

int OccupancyGrid::getWidth()
{
  return width_;
}

int OccupancyGrid::getHeight()
{
  return height_;
}
  
double* OccupancyGrid::getOccupancyMap()
{
  return occupancy_map_;
}

Vector3d OccupancyGrid::getPmin()
{
  return pmin_;
}
Vector3d OccupancyGrid::getPmax()
{
  return pmax_;
}

} // namespace
