#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <time.h>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"

#include "dsl/gridsearch3d.h"
#include "dsl/gridsearch3dvelprm.h"

#include "trimesh2/TriMesh.h"
#include "trimesh2/TriMesh_algo.h"
#include "trimesh2/Vec.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES
#define INFTY 999999999
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define SIGN(a) ((a) >= 0 ? 1 : -1) 

using namespace dsl;
using namespace trimesh;

double haversine_dist(double lat1, double lon1, double lat2, double lon2);

void usage()
{
  std::cout << "usage: dslviz map.stl start_x start_y start_z goal_x goal_y goal_z cells_per_meter samples_per_face [--orient] [--height_penalty] [--georef lat1 lon1 x1 y1 lat2 lon2 x2 y2] [--save_ways outfile lat lon x y] [--show_ways ways.csv] [--show_spline_path max_velocity] [--show_velconstrained_path numVelYaws numVelPitches maxV maxA]" << std::endl; 
}

visualization_msgs::Marker dsl_path_to_ros_marker(const GridPath3D &dsl_path, float cellsPerMeter, float xmin, float ymin, float zmin, float thickness, float r, float g, float b)
{
  visualization_msgs::Marker path_viz;
  std::vector<geometry_msgs::Point> pts;

  float offset = 1.0/(2*cellsPerMeter);
  for(int i = 0; i < dsl_path.count; i++)
  {
    geometry_msgs::Point pt;
    pt.x = dsl_path.pos[3*i]/cellsPerMeter + offset + xmin;//0.5;
    pt.y = dsl_path.pos[3*i+1]/cellsPerMeter + offset + ymin;//0.5;
    pt.z = dsl_path.pos[3*i+2]/cellsPerMeter + offset + zmin;//0.5;
    pts.push_back(pt);
  }

  path_viz.header.frame_id = "/world";
  path_viz.header.stamp = ros::Time();
  path_viz.ns = "dsl";
  path_viz.id = 1;
  path_viz.type = visualization_msgs::Marker::LINE_STRIP;
  path_viz.action = visualization_msgs::Marker::ADD;
  path_viz.pose.position.x = 0;
  path_viz.pose.position.y = 0;
  path_viz.pose.position.z = 0;
  path_viz.pose.orientation.x = 0.0;
  path_viz.pose.orientation.y = 0.0;
  path_viz.pose.orientation.z = 0.0;
  path_viz.pose.orientation.w = 1.0;
  path_viz.scale.x = thickness;
  path_viz.color.a = 1.0;
  path_viz.color.r = r;
  path_viz.color.g = g;
  path_viz.color.b = b;
  path_viz.points = pts;

  return path_viz;  
}

nav_msgs::Path dsl_path_to_ros_msg(const GridPath3D &dsl_path, float cellsPerMeter, float xmin, float ymin, float zmin)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/world";
  msg.poses.resize(dsl_path.count);
  float offset = 1.0/(2*cellsPerMeter);
  for(int i = 0; i < dsl_path.count; i++)
  {
    msg.poses[i].pose.position.x = dsl_path.pos[3*i]/cellsPerMeter + offset + xmin;//0.5;
    msg.poses[i].pose.position.y = dsl_path.pos[3*i+1]/cellsPerMeter + offset + ymin;//0.5;
    msg.poses[i].pose.position.z = dsl_path.pos[3*i+2]/cellsPerMeter + offset + zmin;//0.5;
  }
  return msg; 
}

nav_msgs::Path waycsv_to_xypath(double ref_lat, double ref_lon, Eigen::Vector3d v_ref, std::string filename)
{
  nav_msgs::Path path;
  std::ifstream f;
  std::string line;


  f.open(filename.c_str());
  if(!f.is_open())
  {
    ROS_ERROR("Could not open waypoint file");
    return path;
  }

  int cur_linenum = 1;
  std::getline(f, line); // skip header
  while(std::getline(f, line))
  {
    std::vector<std::string> fields;
    size_t cur_pos=0;
    size_t found_pos=0;
    while((found_pos = line.find(';', cur_pos)) != std::string::npos)
    {
      fields.push_back(line.substr(cur_pos, found_pos-cur_pos));
      cur_pos = found_pos+1;
    }
    if(fields.size() < 3)
    {
      f.close();
      ROS_ERROR("Waypoint file is wrong format: only %zd fields on line %d", fields.size(), cur_linenum);
      return path;
    }
    double lat = atof(fields[0].c_str());
    double lon = atof(fields[1].c_str());
    double dx = SIGN(lon-ref_lon)*haversine_dist(lat, ref_lon, lat, lon);
    double dy = SIGN(lat-ref_lat)*haversine_dist(ref_lat, lon, lat, lon);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = v_ref.x() + dx;
    pose.pose.position.y = v_ref.y() + dy;
    pose.pose.position.z = v_ref.z() + atof(fields[2].c_str())/10. - 1.0;

//    std::cout << pose.pose.position.x << " "
//    << pose.pose.position.y << " "
//    << pose.pose.position.z << " " << std::endl;

    path.poses.push_back(pose);
    cur_linenum++;
  }
  f.close();

  path.header.frame_id = "/world";

  return path;
}

void xypath_to_waycsv(nav_msgs::Path path, double lat, double lon, Eigen::Vector2d v, double zstart_ref, std::string filename)
{
  std::ofstream outf;
  outf.open(filename.c_str());
  if(!outf.is_open())
  {
    std::cout << "Could not open " << filename << std::endl;
    return;
  }

  outf << "WP#;Latitude;Longitude;Height;Heading;Time @ WP; Desired pos. accuracy;Flags;" << std::endl;

  double dlonDx = .0001/haversine_dist(lat,lon,lat,lon+.0001);
  double dlatDy = .0001/haversine_dist(lat,lon,lat+.0001,lon);
  for(unsigned int i = 0; i < path.poses.size(); i++)
  {
    outf << i+1 << ";" 
         << std::fixed << std::setprecision(7) << lat + (path.poses[i].pose.position.y-v.y())*dlatDy << ";" 
         << std::fixed << std::setprecision(7) << lon + (path.poses[i].pose.position.x-v.x())*dlonDx << ";"
         << std::fixed << std::setprecision(3) << (path.poses[i].pose.position.z-zstart_ref)*10 + 10 << ";"
         << "0.000;" << "0.00;" << "3.000;" << "23;" << std::endl;
  }

  outf.close();
}

void xypath_to_waycsv(nav_msgs::Path path, double* times, double lat, double lon, Eigen::Vector2d v, double zstart_ref, std::string filename)
{
  std::ofstream outf;
  outf.open(filename.c_str());
  if(!outf.is_open())
  {
    std::cout << "Could not open " << filename << std::endl;
    return;
  }

  outf << "WP#;Latitude;Longitude;Height;Heading;Time @ WP; Desired pos. accuracy;Flags;Times" << std::endl;

  double dlonDx = .0001/haversine_dist(lat,lon,lat,lon+.0001);
  double dlatDy = .0001/haversine_dist(lat,lon,lat+.0001,lon);
  for(unsigned int i = 0; i < path.poses.size(); i++)
  {
    outf << i+1 << ";" 
         << std::fixed << std::setprecision(7) << lat + (path.poses[i].pose.position.y-v.y())*dlatDy << ";" 
         << std::fixed << std::setprecision(7) << lon + (path.poses[i].pose.position.x-v.x())*dlonDx << ";"
         << std::fixed << std::setprecision(3) << (path.poses[i].pose.position.z-zstart_ref)*10 + 10 << ";"
         << "0.000;" << "0.00;" << "3.000;" << "23;" << times[i] << ";" << std::endl;
  }

  outf.close();
}

void fit_plane_to_map(TriMesh* map, float& nxout, float& nyout, float& nzout, float& x0, float& y0, float& z0)
{
  int numVerts = map->vertices.size();
  float pfailure = .1;
  float poutlier = .90;
  int L = (int)ceil(log(pfailure)/log(1-pow(1-poutlier,3)));
  float reprojThresh = .1;
  int Kthresh = (1-poutlier)*numVerts;
  float nx=0,ny=0,nz=0;
  float x0s=0,y0s=0,z0s=0;
  int Kbest = 0;

  std::vector<Eigen::Vector3d> inliers;
  std::vector<Eigen::Vector3d> bestInliers;
  

  std::cout << "Fitting plane: " << "K=" << Kthresh << " L=" << L << std::endl;

  for(int i = 0; i < L; i++)
  {
    point v1 = map->vertices[rand()%numVerts];
    point v2 = map->vertices[rand()%numVerts];
    point v3 = map->vertices[rand()%numVerts];
    
    float ux = v2[0]-v1[0];
    float uy = v2[1]-v1[1];
    float uz = v2[2]-v1[2];
    float vx = v3[0]-v1[0];
    float vy = v3[1]-v1[1];
    float vz = v3[2]-v1[2];

    nx = uy*vz-uz*vy;
    ny = uz*vx-ux*vz;
    nz = ux*vy-uy*vx;
    float n = sqrt(nx*nx + ny*ny + nz*nz);
    nx /= n;
    ny /= n;
    nz /= n;
    
    x0s = v1[0];
    y0s = v1[1];
    z0s = v1[2];
    
    int K = 0;
    inliers.clear();
    for(int j = 0; j < numVerts; j++)
    {
      float x = map->vertices[j][0]-x0s;    
      float y = map->vertices[j][1]-y0s;    
      float z = map->vertices[j][2]-z0s;
      
      //std::cout << fabs(x*nx + y*ny + z*nz) << std::endl;
      if(fabs(x*nx + y*ny + z*nz) < reprojThresh)
      {
        inliers.push_back(Eigen::Vector3d(x,y,z));
        K++;
      }    
    }
    if(K > Kthresh && K > Kbest)
    {
      Kbest = K;
      bestInliers = inliers;
      x0 = x0s;
      y0 = y0s;
      z0 = z0s;
      nxout = nx;
      nyout = ny;
      nzout = nz;
    }
  }
  std::cout << "Kbest: " << Kbest << std::endl;

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d meanSqr = Eigen::Matrix3d::Zero();

  for(size_t i = 0; i < bestInliers.size(); i++)
  {
    mean += bestInliers[i];
    meanSqr += bestInliers[i]*bestInliers[i].transpose();
  }
  mean /= bestInliers.size();
  meanSqr /= bestInliers.size();

  Eigen::Matrix3d cov = meanSqr - mean*mean.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  Eigen::Vector3d n = eig.eigenvectors().col(0);

  //std::cout << "refit n: " << n << std::endl;
  nxout = n[0];
  nyout = n[1];
  nzout = n[2];

  // Normal always up
  if(nzout < 0)
  {
    nxout *= -1;
    nyout *= -1;
    nzout *= -1;
  } 

}


void normal_to_angle(float nx, float ny, float nz, float& vx, float& vy, float& vz, float& t)
{ 
  vx = ny;
  vy = -nx;
  vz = 0;
  t = acos(nz);
}

void normal_to_quat(float nx, float ny, float nz, float& qx, float& qy, float& qz, float& qw)
{
  Eigen::Vector3d z(0,0,1);
  Eigen::Vector3d n(nx,ny,nz);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(n,z);
  qx = q.x();
  qy = q.y();
  qz = q.z();
  qw = q.w();
}

void sample_point_in_triangle(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, float& xout, float& yout, float& zout)
{
  float ux = x2-x1;
  float uy = y2-y1;
  float uz = z2-z1;
  float vx = x3-x1;
  float vy = y3-y1;
  float vz = z3-z1;

  float nx = uy*vz-uz*vy;
  float ny = uz*vx-ux*vz;
  float nz = ux*vy-uy*vx;

  float x = float(rand())/RAND_MAX;
  float y = float(rand())/RAND_MAX;
  while(x + y > 1)
  {
    x = float(rand())/RAND_MAX;
    y = float(rand())/RAND_MAX;
  }
  
  float xs = ux*x + vx*y;
  float ys = uy*x + vy*y;
  float zs;
  if(nz > 1e-6)
  {
    zs = (-nx*xs - ny*ys)/nz;
    xout = xs+x1;
    yout = ys+y1;
    zout = zs+z1;
  }
  else
  {
    xout = x1;
    yout = y1;
    zout = z1;
  }

}

double haversine_dist(double lat1, double lon1, double lat2, double lon2)
{
  double R = 637100; // radius of Earth (10m)
  double phi1 = lat1 *M_PI/180;
  double phi2 = lat2 *M_PI/180;
  double dphi = (lat2-lat1) *M_PI/180;
  double dlamda = (lon2-lon1) *M_PI/180;
  double a = sin(dphi/2)*sin(dphi/2) + cos(phi1)*cos(phi2) * sin(dlamda/2)*sin(dlamda/2);
  double c = 2*atan2(sqrt(a),sqrt(1-a));

  return R*c;
}

double latlon_to_scale(double lat1, double lon1, Eigen::Vector2d v1, double lat2, double lon2, Eigen::Vector2d v2)
{
  double current_d = (v2-v1).norm();
  return haversine_dist(lat1, lon1, lat2, lon2)/current_d; 
}

double latlon_to_zrot(double lat1, double lon1, Eigen::Vector2d v1, double lat2, double lon2, Eigen::Vector2d v2)
{
  float dx = SIGN(lon2-lon1)*haversine_dist(lat1,lon1,lat1,lon2);
  float dy = SIGN(lat2-lat1)*haversine_dist(lat1,lon1,lat2,lon1);

  float tll = atan2(dy,dx);
  float txy = atan2(v2.y()-v1.y(),v2.x()-v1.x());

  return tll-txy;
}
 
int main(int argc, char **argv)
{
  if(argc < 10)
  {
    usage();
    return -1;
  }
  char* map_filename = argv[1];
  float startx = atof(argv[2]);
  float starty = atof(argv[3]); 
  float startz = atof(argv[4]);
  float goalx = atof(argv[5]);
  float goaly = atof(argv[6]);
  float goalz = atof(argv[7]);
  float cellsPerMeter = atof(argv[8]); 
  unsigned int samplesPerFace = atoi(argv[9]);
  bool textured = false;
  bool orient = false;
  bool height_penalty = false;
  bool geo_ref = false;
  bool save_ways = false;
  bool show_ways = false;
  bool show_spline_path = false;
  bool show_velconstrained_path = false;
  double markerThickness = .05;
  double numVelYaws=0, numVelPitches=0, velMax=0, accMax=0;

  double spline_path_maxvelocity=0;

  double geo_lat1=0,geo_lon1=0,geo_lat2=0,geo_lon2=0;
  Eigen::Vector2d geo_v1;
  Eigen::Vector2d geo_v2;

  double ways_lat=0, ways_lon=0;
  Eigen::Vector2d ways_v;
  std::string ways_filename;

  std::string waystoshow_filename;

  int length, width, height;
  float map_nx, map_ny, map_nz, map_x0, map_y0, map_z0;
  float qx,qy,qz,qw;
  float xmin, xmax, ymin, ymax, zmin, zmax;
  double zstart_ref = INFTY;
  
  int numOccupiedCells = 0;
  
  GridPath3D dslpath, optpath;
  GridPath3DPlusTime splinepath, splineoptpath, velpath;
  std::vector<geometry_msgs::Point> marker_pos;
  
  for(int i = 10; i < argc; i++)
  {
    if(std::string(argv[i]) == "--orient")
      orient = true;
    else if(std::string(argv[i]) == "--height_penalty")
      height_penalty = true;
    else if(std::string(argv[i]) == "--georef")
    {
      if(i + 8 >= argc)
      {
        std::cout << "Not enough arguments for --georef" << std::endl;
        return -1;
      }
      geo_lat1 = atof(argv[i+1]);
      geo_lon1 = atof(argv[i+2]);
      geo_v1 = Eigen::Vector2d(atof(argv[i+3]), atof(argv[i+4]));
      geo_lat2 = atof(argv[i+5]);
      geo_lon2 = atof(argv[i+6]);
      geo_v2 = Eigen::Vector2d(atof(argv[i+7]), atof(argv[i+8]));
      geo_ref = true;
      i += 8;
    }
    else if(std::string(argv[i]) == "--save_ways")
    {
      if(i + 5 >= argc)
      {
        std::cout << "Not enough arguments for --save_ways" << std::endl;
        return -1;
      }
      ways_filename = std::string(argv[i+1]);
      ways_lat = atof(argv[i+2]);
      ways_lon = atof(argv[i+3]);
      ways_v = Eigen::Vector2d(atof(argv[i+4]), atof(argv[i+5]));
      save_ways = true;
      i += 5;
    }
    else if(std::string(argv[i]) == "--show_ways")
    {
      if(i + 1 >= argc)
      {
        std::cout << "Not enough arguments for --show_ways" << std::endl;
        return -1;
      }
      show_ways = true;
      waystoshow_filename = std::string(argv[i+1]);
      i += 1;
    }
    else if(std::string(argv[i]) == "--show_spline_path")
    {
      if(i + 1 >= argc)
      {
        std::cout << "Not enough arguments for --show_spline_path" << std::endl;
        return -1;
      }
      show_spline_path = true;
      spline_path_maxvelocity = atof(argv[i+1]);
      i += 1;
    }
    else if(std::string(argv[i]) == "--show_velconstrained_path")
    {
      if(i + 4 >= argc)
      {
        std::cout << "Not enough arguments for --show_velconstrained_path" << std::endl;
        return -1;
      }
      show_velconstrained_path = true;
      numVelYaws = atof(argv[i+1]);
      numVelPitches = atof(argv[i+2]);
      velMax = atof(argv[i+3]);
      accMax = atof(argv[i+4]);
      i += 4;
    }
    else if(std::string(argv[i]) == "--textured")
    {
      textured = true;
    }
    else
    {
      std::cout << "Option \"" << argv[i] << "\" not recognized" << std::endl;
      return -1;
    }
  }

  
  srand(time(NULL));
  
  //Read in mesh and compute length, width, height
  TriMesh *map = TriMesh::read(map_filename);
  if(!map)
  {
    std::cout << "Could not open mesh: " << map_filename << std::endl;
    return -1;
  }
  map->need_faces();
  
  if(orient)
  {
    fit_plane_to_map(map, map_nx, map_ny, map_nz, map_x0, map_y0, map_z0);
    std::cout << "Map normal: (" << map_nx << "," << map_ny << "," << map_nz << ")" << std::endl;
    std::cout << "Plane center: (" << map_x0 << "," << map_y0 << "," << map_z0 << ")" << std::endl;

    vec map_trans(-map_x0,-map_y0,-map_z0);
    trans(map, map_trans); 
    
    float vx,vy,vz,t;

    normal_to_quat(map_nx, map_ny, map_nz, qx, qy, qz, qw);
    normal_to_angle(map_nx, map_ny, map_nz, vx, vy, vz, t);
    vec map_rot(vx,vy,vz);
    rot(map, t, map_rot); 
    std::cout << vx << " " << vy << " " << vz << " " << t << std::endl;
    
    map->write(std::string(map_filename));
  }

  if(geo_ref)
  {
    scale(map, latlon_to_scale(geo_lat1, geo_lon1, geo_v1, geo_lat2, geo_lon2, geo_v2));
    rot(map, latlon_to_zrot(geo_lat1, geo_lon1, geo_v1, geo_lat2, geo_lon2, geo_v2), vec(0,0,1));
    
    map->write(std::string(map_filename));
  }

  xmin = ymin = zmin = INFTY;
  xmax = ymax = zmax = -INFTY;
  for(unsigned int i = 0; i < map->vertices.size(); i++)
  {
    xmin = MIN(map->vertices[i][0],xmin);
    ymin = MIN(map->vertices[i][1],ymin);
    zmin = MIN(map->vertices[i][2],zmin);
    xmax = MAX(map->vertices[i][0],xmax);
    ymax = MAX(map->vertices[i][1],ymax);
    zmax = MAX(map->vertices[i][2],zmax);
  }


  std::cout << "Num vertices: " << map->vertices.size() << std::endl;
  std::cout << "Min coords: (" << xmin << "," << ymin << "," << zmin << ")" << " Max coords: (" << xmax << "," << ymax << "," << zmax << ")" << std::endl;


  if(startx > xmax || starty > ymax || startz > zmax || startx < xmin || starty < ymin || startz < zmin)
  {  
    std::cout << "Error: starting coordinate out of bounds" << std::endl;
    return -1;
  }
  
  if(goalx > xmax || goaly > ymax || goalz > zmax || goalx < xmin || goaly < ymin || goalz < zmin)
  {  
    std::cout << "Error: goal coordinate out of bounds" << std::endl;
    return -1;
  }

  length = (int)ceil((xmax - xmin)*cellsPerMeter);
  width = (int)ceil((ymax - ymin)*cellsPerMeter);
  height = (int)ceil((zmax - zmin)*cellsPerMeter);

  std::cout << "lwh: " << length << " " << width << " " << height << std::endl;

  // create an occupancy map
  double *occupancy_map = new double[length*width*height];
  if(!occupancy_map)
  {
    std::cout << "Failed to malloc occupancy map" << std::endl;
    return 0;
  }

  int start_idx = (int)floor((startx - xmin)*cellsPerMeter) + ((int)floor((starty - ymin)*cellsPerMeter))*length;
  for(unsigned int i = 0; i < map->vertices.size(); i++)
  {   
    int x = (int)floor((map->vertices[i][0] - xmin)*cellsPerMeter);
    int y = (int)floor((map->vertices[i][1] - ymin)*cellsPerMeter);
    double z = map->vertices[i][2];
    int idx = x + y*length;
    if(idx == start_idx)
      zstart_ref = MIN(z, zstart_ref);
  }
  std::cout << "zstart_ref: " << zstart_ref << std::endl;

  if(height_penalty)
  {  
    double *minheight_map = new double[length*width];
    for(int i = 0; i < length*width; i++)
    {
      minheight_map[i] = INFTY;
    }  
  
    for(unsigned int i = 0; i < map->vertices.size(); i++)
    {
      
      int x = (int)floor((map->vertices[i][0] - xmin)*cellsPerMeter);
      int y = (int)floor((map->vertices[i][1] - ymin)*cellsPerMeter);
      int z = (int)floor((map->vertices[i][2] - zmin)*cellsPerMeter);
      int idx = x + y*length;
      if(idx >= length*width || idx < 0) std::cout << "Out of bounds: " << x << "," << y << "," << z << std::endl;
      minheight_map[idx] = MIN(z,minheight_map[idx]);
    }

    //Initialize map, penalize greater heights
    for(int i = 0; i < length*width*height; i++)
    {
      int z = (int)floor(i / (length*width));
      int idx = i - z*length*width;
      if(z <= minheight_map[idx])
        occupancy_map[i] = 2*INFTY;
      else
        occupancy_map[i] = z;
    }

    delete[] minheight_map;
  }
  else
  {
    for(int i = 0; i < length*width*height; i++)
    {
      occupancy_map[i] = 0;
    }
  }

  for(unsigned int i = 0; i < map->vertices.size(); i++)
  {
    int x = (int)floor((map->vertices[i][0] - xmin)*cellsPerMeter);
    int y = (int)floor((map->vertices[i][1] - ymin)*cellsPerMeter);
    int z = (int)floor((map->vertices[i][2] - zmin)*cellsPerMeter);
    int idx = x + y*length + z*length*width;
    if(idx >= length*width*height || idx < 0) std::cout << "Out of bounds: " << x << "," << y << "," << z << std::endl;
    if(occupancy_map[idx] != INFTY)
    {
      //std::cout << x << "," << y << "," << z << std::endl;
      occupancy_map[idx] = INFTY;
      geometry_msgs::Point pt;
      pt.x = x/cellsPerMeter;
      pt.y = y/cellsPerMeter;
      pt.z = z/cellsPerMeter;
      marker_pos.push_back(pt);
      numOccupiedCells++;
    }
  }

  // Sample points on mesh faces
  for(unsigned int i = 0; i < map->faces.size(); i++)
  {
    point v1 = map->vertices[map->faces[i][0]];
    point v2 = map->vertices[map->faces[i][1]];
    point v3 = map->vertices[map->faces[i][2]];
    for(unsigned int j = 0; j < samplesPerFace; j++)
    {
      float xs;
      float ys;
      float zs;
      sample_point_in_triangle(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2], xs, ys, zs);
      int x = (int)floor((xs - xmin)*cellsPerMeter);
      int y = (int)floor((ys - ymin)*cellsPerMeter);
      int z = (int)floor((zs - zmin)*cellsPerMeter);
      
      int idx = x + y*length + z*length*width;
      if(idx >= length*width*height || idx < 0) std::cout << "Sample out of bounds: " << x << "," << y << "," << z << " sample: " << xs << "," << ys << "," << zs << std::endl;
      if(occupancy_map[idx] != INFTY)
      {
        //std::cout << x << "," << y << "," << z << std::endl;
        occupancy_map[idx] = INFTY;
        geometry_msgs::Point pt;
        pt.x = x/cellsPerMeter;
        pt.y = y/cellsPerMeter;
        pt.z = z/cellsPerMeter;
        marker_pos.push_back(pt);
        numOccupiedCells++;
      }
    }
  }

  std::cout << "Num Occupied Cells: " << numOccupiedCells << std::endl;
  std::cout << "% Occupied Cells: " << 100*float(numOccupiedCells)/(length*width*height) << std::endl; 
  if(numOccupiedCells > 10000)
   std::cout << "Warning: Rviz may not properly visualize >10000 cells" << std::endl;
  
  //Perform dsl gridsearch3D
  GridSearch3D gdsl(length, width, height, occupancy_map);
  gdsl.SetStart((startx - xmin)*cellsPerMeter, (starty - ymin)*cellsPerMeter, (startz - zmin)*cellsPerMeter);
  gdsl.SetGoal((goalx - xmin)*cellsPerMeter, (goaly - ymin)*cellsPerMeter, (goalz - zmin)*cellsPerMeter);
  gdsl.Plan(dslpath);
  gdsl.OptPath(dslpath, optpath);
  
  if(show_spline_path)
  {
    gdsl.SmoothPathOptCost(dslpath, splinepath, spline_path_maxvelocity*cellsPerMeter/10., .4);
    gdsl.SmoothPathSpline(optpath, splineoptpath, spline_path_maxvelocity*cellsPerMeter/10., .4);
  }

  if(show_velconstrained_path)
  {
    double v = velMax;
    double a = accMax;
    double vmin[3] = {-v*cellsPerMeter/10., -v*cellsPerMeter/10., -v*cellsPerMeter/10.};
    double vmax[3] = {v*cellsPerMeter/10., v*cellsPerMeter/10., v*cellsPerMeter/10.};
    double amin[3] = {-a*cellsPerMeter/10., -a*cellsPerMeter/10., -a*cellsPerMeter/10.};
    double amax[3] = {a*cellsPerMeter/10., a*cellsPerMeter/10., a*cellsPerMeter/10.};
    
    GridSearch3DVelPRM gdsl_v(length, width, height, numVelYaws, numVelPitches, cellsPerMeter/10., vmin, vmax, amin, amax, occupancy_map);
    gdsl_v.Init();
    gdsl_v.SetStart((startx - xmin)*cellsPerMeter, (starty - ymin)*cellsPerMeter, (startz - zmin)*cellsPerMeter, 1, 0);
    gdsl_v.SetGoal((goalx - xmin)*cellsPerMeter, (goaly - ymin)*cellsPerMeter, (goalz - zmin)*cellsPerMeter, 1, 0);
    gdsl_v.Plan(velpath);
  }


  std::cout << "Done planning" << std::endl;

  // Publish to ROS
  ros::init(argc, argv, "dslviz");

  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/dsl/path", 1000);
  ros::Publisher path_marker_pub = n.advertise<visualization_msgs::Marker>("/dsl/path_marker", 1000);
  nav_msgs::Path path = dsl_path_to_ros_msg(dslpath, cellsPerMeter, xmin, ymin, zmin);

  ros::Publisher velpath_pub = n.advertise<nav_msgs::Path>("/dsl/velpath", 1000);
  ros::Publisher velpath_marker_pub = n.advertise<visualization_msgs::Marker>("/dsl/velpath_marker", 1000);
  nav_msgs::Path velpath_path;
  
  ros::Publisher optpath_pub = n.advertise<nav_msgs::Path>("/dsl/optpath", 1000);
  ros::Publisher optpath_marker_pub = n.advertise<visualization_msgs::Marker>("/dsl/optpath_marker", 1000);
  nav_msgs::Path optpath_path;
  
  ros::Publisher smoothpath_pub = n.advertise<nav_msgs::Path>("/dsl/smoothpath", 1000);
  ros::Publisher smoothpath_marker_pub = n.advertise<visualization_msgs::Marker>("/dsl/smoothpath_marker", 1000);
  nav_msgs::Path smoothpath_path;
  
  ros::Publisher actual_path_pub = n.advertise<nav_msgs::Path>("/dsl/actual_path", 1000);
  nav_msgs::Path actual_path;


  ros::Rate loop_rate(1);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0.0, 0, 0);
  transform.setRotation(q);
  
  
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "/dsl/map_mesh", 0 );
  ros::Publisher plane_vis_pub = n.advertise<visualization_msgs::Marker>( "/dsl/fit_plane", 0 );
  ros::Publisher occ_map_vis_pub = n.advertise<visualization_msgs::Marker>( "/dsl/occupancy_map", 0 );
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time();
  marker.ns = "dsl";
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
  if(textured)
  {
    marker.color.a = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_use_embedded_materials = true;

    std::string map_fn(map_filename);
    unsigned int found = map_fn.find_last_of(".");
    std::string texture_fn =  std::string("package://dslviz/") + map_fn.substr(0,found) + std::string(".dae");
    marker.mesh_resource = texture_fn;
    std::cout << "Using textured mesh: " << texture_fn << std::endl;
  }
  else
  {
    marker.mesh_resource = std::string("package://dslviz/") + std::string(map_filename);
  }
 /* 
  visualization_msgs::Marker plane;
  plane.header.frame_id = "/world";
  plane.header.stamp = ros::Time();
  plane.ns = "dsl";
  plane.id = 0;
  plane.type = visualization_msgs::Marker::CUBE;
  plane.action = visualization_msgs::Marker::ADD;
  plane.pose.position.x = -xmin+map_x0;
  plane.pose.position.y = -ymin+map_y0;
  plane.pose.position.z = -zmin+map_z0;
  plane.pose.orientation.x = qx;//0.0;
  plane.pose.orientation.y = qy;//0.0;
  plane.pose.orientation.z = qz;//0.0;
  plane.pose.orientation.w = qw;//1.0;
  plane.scale.x = xmax-xmin;
  plane.scale.y = ymax-ymin;
  plane.scale.z = 0.1;
  plane.color.a = 0.5;
  plane.color.r = 0.0;
  plane.color.g = 0.0;
  plane.color.b = 1.0;
*/
  visualization_msgs::Marker occmap_viz;
  
  occmap_viz.header.frame_id = "/world";
  occmap_viz.header.stamp = ros::Time();
  occmap_viz.ns = "dsl";
  occmap_viz.id = 1;
  occmap_viz.type = visualization_msgs::Marker::CUBE_LIST;
  occmap_viz.action = visualization_msgs::Marker::ADD;
  occmap_viz.pose.position.x = 1.0/(2.*cellsPerMeter) + xmin;//0.5;
  occmap_viz.pose.position.y = 1.0/(2.*cellsPerMeter) + ymin;//0.5;
  occmap_viz.pose.position.z = 1.0/(2.*cellsPerMeter) + zmin;//0.5;
  occmap_viz.pose.orientation.x = 0.0;
  occmap_viz.pose.orientation.y = 0.0;
  occmap_viz.pose.orientation.z = 0.0;
  occmap_viz.pose.orientation.w = 1.0;
  occmap_viz.scale.x = 1.0/cellsPerMeter;
  occmap_viz.scale.y = 1.0/cellsPerMeter;
  occmap_viz.scale.z = 1.0/cellsPerMeter;
  occmap_viz.color.a = 0.5;
  occmap_viz.color.r = 1.0;
  occmap_viz.color.g = 0.0;
  occmap_viz.color.b = 0.0;
  occmap_viz.points = marker_pos;

  std::cout << "Cube scale: " << occmap_viz.scale.x << std::endl;

  if(save_ways)
  {
    xypath_to_waycsv(path, ways_lat, ways_lon, ways_v, zstart_ref, ways_filename);
  }
  std::cout << "Waiting for subscriber before exit..." << std::endl;
  
  while (path_pub.getNumSubscribers() == 0)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(show_ways)
  {
    Eigen::Vector3d v_ref(ways_v.x(), ways_v.y(), zstart_ref);
    actual_path = waycsv_to_xypath(ways_lat, ways_lon, v_ref, waystoshow_filename);
    actual_path_pub.publish(actual_path);
    std::cout << "Published actual path from file" << std::endl;
  }

  if(show_spline_path)
  {
    optpath_path = dsl_path_to_ros_msg(splineoptpath,cellsPerMeter, xmin, ymin, zmin);
    smoothpath_path = dsl_path_to_ros_msg(splinepath, cellsPerMeter, xmin, ymin, zmin);
    smoothpath_pub.publish(smoothpath_path);
    smoothpath_marker_pub.publish(dsl_path_to_ros_marker(splinepath, cellsPerMeter, xmin, ymin, zmin, markerThickness, 1.0, 0.5, 1.0));
    if(save_ways)
    {
      xypath_to_waycsv(optpath_path, splineoptpath.times, ways_lat, ways_lon, ways_v, zstart_ref, ways_filename + "_optspline");
      xypath_to_waycsv(smoothpath_path, splinepath.times, ways_lat, ways_lon, ways_v, zstart_ref, ways_filename + "_spline");
    }
    optpath_marker_pub.publish(dsl_path_to_ros_marker(splineoptpath, cellsPerMeter, xmin, ymin, zmin, markerThickness, 0.0, 0.5, 1.0));
  }
  else
  {
    optpath_path = dsl_path_to_ros_msg(optpath,cellsPerMeter, xmin, ymin, zmin);
    if(save_ways)
    {
      xypath_to_waycsv(optpath_path, ways_lat, ways_lon, ways_v, zstart_ref, ways_filename + "_opt");
    }
    optpath_marker_pub.publish(dsl_path_to_ros_marker(optpath, cellsPerMeter, xmin, ymin, zmin, markerThickness, 0.0, 0.5, 1.0));
  }

  if(show_velconstrained_path)
  {
    velpath_path = dsl_path_to_ros_msg(velpath, cellsPerMeter, xmin, ymin, zmin);
    velpath_pub.publish(velpath_path);
    velpath_marker_pub.publish(dsl_path_to_ros_marker(velpath, cellsPerMeter, xmin, ymin, zmin, markerThickness, 1.0, 1.0, 0.0));
    if(save_ways)
    {
      xypath_to_waycsv(velpath_path, velpath.times, ways_lat, ways_lon, ways_v, zstart_ref, ways_filename + "_velconstrained");
    }
  }

  path_pub.publish(path);
  path_marker_pub.publish(dsl_path_to_ros_marker(dslpath, cellsPerMeter, xmin, ymin, zmin, markerThickness, 0.0, 1.0, 0.0));
  optpath_pub.publish(optpath_path);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dsl"));
  vis_pub.publish( marker );
  //plane_vis_pub.publish( plane );
  occ_map_vis_pub.publish( occmap_viz );
    
  delete[] occupancy_map;
  return 0;
}
