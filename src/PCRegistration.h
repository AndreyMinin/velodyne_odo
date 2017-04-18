/*
 * PCRegistration.h
 *
 *  Created on: 19.11.2014
 *      Author: user
 */

#ifndef PCREGISTRATION_H_
#define PCREGISTRATION_H_

#include "Map3D.h"
#include "SphereArr.h"
#include <velodyne_pointcloud/rawdata.h>
#include <tf/LinearMath/Transform.h>
#include "QTransform.h"
#include "vcloud.h"

//#define DEBUG_STEP
#ifdef DEBUG_STEP
#include <fstream>
#endif

namespace registration
{


//default value for cells out of spheres
const double EMPTY_CELL = 1000000;

typedef struct cell
{
  typedef double v_value_t;
  //vector to center
  v_value_t x;
  v_value_t y;
  v_value_t z;
  //range from nearest center, (EMPTY_CELL) - out of nearest point sphere
  //or vector modul value
  double range;
  cell(double r = EMPTY_CELL):
    x(0), y(0), z(0), range(r){}
  cell(v_value_t i, v_value_t j, v_value_t k, double r):
    x(i), y(j), z(k), range(r){}
  cell(const cell& right):
    x(right.x), y(right.y), z(right.z), range(right.range){}
  bool operator < (const cell& right) const
  { return range < right.range; }
  cell& operator = (const cell& right)
  { x = right.x; y = right.y; z = right.z;
    range = right.range; return *this;}
  cell&  operator += (const cell& right)
  {
    x += right.x; y += right.y; z += right.z;
//    range = sqrt(x*x + y*y + z*z);
    if ( range > right.range )
      range = right.range;
    return *this;
  }
}cell_t;

/*
 * @brief: functor returns cell_t filled according to displacement from sphere center
 */
class Modul
{
public :
  /*
   * @brief
   * @param i,j, k index displacement from sphere center
   * @param r - range from center
   */
  cell_t operator ()(int i, int j, int k, double r)
  {
    if ( !i && !j && !k )
      return cell_t(0,0,0,r);
    double mod = sqrt(i*i + j*j + k*k);
    return cell_t((double)(-i)/mod/r, (double)(-j)/mod/r, (double)(-k)/mod/r, r);
  }
};


class Quadric
{
protected:
  double rmax;
public :
  /*
   *@param rmax - sphere radius
   */
  Quadric(double max):rmax(max){}
  cell_t operator ()(int i, int j, int k,  double r)
  {
    if ( !i && !j && !k )
        return cell_t(0,0,0,r);
    double mod = sqrt(i*i + j*j + k*k);
    double val = r*r / rmax;
    return cell_t((double)(-i)/mod*val, (double)(-j)/mod*val, (double)(-k)/mod*val, r);
  }
};

class InvQuadric //f = 1 / r^2
{
protected:
  double rmax;
public:
  InvQuadric(double max) :
      rmax(max)
  {
  }
  cell_t operator()(int i, int j, int k, double r)
  {
    if (!i && !j && !k)
      return cell_t(0, 0, 0, r);
    double mod = sqrt(i * i + j * j + k * k);
    double val = 1.0 / (r * r); //rmax - r*r / rmax;
    return cell_t((double)(-i) / mod * val, (double)(-j) / mod * val, (double)(-k) / mod * val, r);
  }
};

class Gaussian //f = fmax*exp(-x^2/sigma)
{
protected:
    double Fmax;
    double sigma_sqr;
public:
    Gaussian(double f, double s): Fmax(f),sigma_sqr(s*s){}
    cell_t operator()(int i, int j, int k, double r)
    {
        if (!i && !j && !k)
          return cell_t(0, 0, 0, r);
        double mod = sqrt(i * i + j * j + k * k);
        double val = Fmax * exp(-(r * r)/sigma_sqr);
        return cell_t((double)(-i) / mod * val, (double)(-j) / mod * val, (double)(-k) / mod * val, r);
    }
};

class PCRegistration
{

protected:
  Map3D<double, cell_t> map;
  double point_field_radius;
  typedef SphereArr<double, cell_t> Sphere;
  //force field for point
  Sphere point_sphere;
  //clear field for point
  Sphere clear_sphere;
  //forces
  std::vector<tf::tfVector4> force_vectors;
  //corresponding application points
  std::vector<tf::Vector3> force_points;
  int forces_number;
  bool findStepTransform(const VPointCloud::Ptr &scan, const QTransform& transform, QTransform& step);
//  double findStepRotation(const VPointCloud::Ptr& scan, const QTransform& transform, QTransform& step);
//  double findStepTranslation(const VPointCloud::Ptr& scan, const QTransform& transform, QTransform& step);
  //finds vector pointing to max likelihood for point 4th coordinate is range to point
  tf::tfVector4 findPointVector(double x, double y, double z);
  tf::tfVector4 findPointVector(tf::Vector3& point);
  //finds transform
  tf::Vector3 findEqualDisplacement();
  tf::Quaternion findEqualRotation();
  tf::Vector3 findEqualRotationVector();
  double MAX_DISPLACEMENT ; //m
  double MAX_ROTATION ; //rad
  double MIN_DISPLACEMENT;
  double MIN_ROTATION ;
  size_t max_iterations_number;
  //for adding force field
  bool addPoint(const tf::Vector3& v)
  { return  map.addSphere(v.x(), v.y(), v.z(), point_sphere); }
  bool addPoint(float x, float y, float z)
  { return  map.addSphere(x, y, z, point_sphere); }
  void addPointFast(float x, float y, float z)
  { map.addSphereFast(x, y, z, point_sphere); }
  //for setting points under sphere
  void setPointFast(float x, float y, float z)
  { map.setSphereFast(x, y, z, clear_sphere); }

  bool setPoint(float x, float y, float z)
  { return  map.setSphere(x, y, z, clear_sphere); }
  //returns true if step is possible
  //step is possible if value in (transform + old_step) < old_value
  //fills value with new one if step is possible
  bool testStepTransform(const VPointCloud::Ptr &scan,
                             const QTransform& transform,
                             const QTransform& step,
                             double& value);
  QTransform calculateStepRotation(const VPointCloud::Ptr &scan,
                              const QTransform& transform);
  QTransform calculateStepTranslation(const VPointCloud::Ptr &scan,
                              const QTransform& transform);
  QTransform calculateStep(const VPointCloud::Ptr &scan,
                                const QTransform& transform);

  //returns error value of scan displacement in map
  double calculateForces(const VPointCloud::Ptr &scan,const QTransform& transform);
  //calculates step_rotation_vector and step_translation_vector
  void calculateTransformVectors();
  //
  void applyStepTransform( QTransform& transform,
                          const QTransform& step);

  //decrease step than sum of transforms became smaller than threshold
  void decreaseStepbySum();
  //decrease step than transform rotates more than threshold
  void decreaseStepbyAngle();

  bool decreaseTranslation();
  bool decreaseRotation();

  void sumTransform(const QTransform& step);
  double step_rotation;
  double step_translation;
  double weight_sum;
  tf::Vector3 step_rotation_vector;
  tf::Vector3 step_translation_vector;
  tf::Vector3 old_step_translation ;
  tf::Vector3 old_step_rotation;
  std::list<tf::Vector3> last_translations;
  std::list<tf::Quaternion> last_rotations;

  tf::Vector3 translation_sum;
  tf::Quaternion rotation_sum;

  int translation_sum_num;
  int rotation_sum_num;
  //number of step to calculate termination sum
  int max_sum_num;
#ifdef DEBUG_STEP
  //files to save search trajectory in one step
  std::ofstream out_v;
  std::ofstream out_w;
#endif

public:
  typedef Map3D<double, cell_t> RegMapT;
  RegMapT & getMap(){ return map; }
  void clear(){ map.zero();}

  void reset(double x0, double y0, double z0)
  {
    map.zero();
    map.setOrigin(x0, y0, z0);
  }

  void reset()
  {
    map.zero();
  }
  /*@brief constructor
   * @param origX, @origY @origZ - origin of map
   * @param sizeX - size of grid
   * @param dx - granularity
   * @param radius sphere radius then points placed in to the map
   * @param miss_set - miss number of scans when set scan to map
   * @param miss_find - miss number of scans when find transform
   */
  PCRegistration(double origX, double origY, double origZ,
                 double sizeX, double sizeY, double sizeZ,
                 double dx, double radius,
                 size_t max_iter = 200,
                 double max_dis =  0.005, //m
                 double max_rot = 0.001, //rad
                 double min_dis = 0.0001,
                 double min_rot = 0.0001);

  /*
   * @brief insert point cloud into map as force field
   * @param scan - laser data
   */
  void setPointCloud(const VPointCloud::Ptr &scan);
  /*
   *@brief insert point cloud into map without limits check
   *@param scan - laser data
   */
  void setPointCloudFast(const VPointCloud::Ptr &scan);
  /*
   *@brief set scan in to map according to transform
   *@param scan - laser data
   *@param transform - rotation and position relative zero point
   */
  void setPointCloud(const VPointCloud::Ptr &scan, const QTransform& transform);
  void setPointCloud(const VPointCloud::Ptr &scan, const tf::Transform& transform)
  {
    setPointCloud(scan, QTransform(transform));
  }

  /*
   *@brief clear map points according to scan
   *@param scan - laser data
   */
  void clearPointCloud(const VPointCloud::Ptr &scan);
  void clearPointCloudFast(const VPointCloud::Ptr &scan);

  /*@brief finds transform for alignment of scan to map
   *@return number of steps or -1 if failed
   */
  int findTransform(const VPointCloud::Ptr &scan, QTransform& transform);

  int findTransform(const VPointCloud::Ptr &scan, tf::Transform& transform)
  {
    QTransform qtransform(transform);
    int res = findTransform(scan, qtransform);
    transform = qtransform.transform();
    return res;
  }

  double activePointsQuantity() { return forces_number; }

  SphereArr<double, cell_t>& getSphere(){ return point_sphere;  }

  void getHorisontalSlice(std::vector<int8_t>& output_array, double h);

};

} /* namespace registration */
#endif /* PCREGISTRATION_H_ */
