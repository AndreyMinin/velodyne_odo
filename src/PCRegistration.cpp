/*
 * PCRegistration.cpp
 *
 *  Created on: 19.11.2014
 *      Author: user
 */

#include "PCRegistration.h"
#include <vector>
#include <ros/ros.h>


namespace registration
{

void PCRegistration::getHorisontalSlice(std::vector<int8_t>& output_array, double h)
{
  int z = map.z2Index(h);
  output_array.resize(map.cellSizeX()*map.cellSizeY());
  //InvQuadric func(point_sphere.maximum());

  //cell_t max_cell = func(1,0,0,map.resolution());

  double max = 10;//10*sqrt( max_cell.x * max_cell.x + max_cell.y * max_cell.y + max_cell.z * max_cell.z);
      //point_sphere.maximum();
//  ROS_INFO_STREAM("slice max : "<<max);
  for( int i = 0; i<map.cellSizeX(); i++)
  {
    for( int j = 0; j<map.cellSizeY(); j++)
    {
      cell_t cell = map(i, j, z);
      double value = //cell.range;
          sqrt( cell.x * cell.x + cell.y * cell.y + cell.z * cell.z);

      output_array[i + j*map.cellSizeY()] = (value > EMPTY_CELL-1)? 0: value*100.0 / max;
    }
  }
}

double PCRegistration::calculateForces(const VPointCloud::Ptr &scan,
                                            const QTransform& transform)
{
  forces_number = 0;
  double sum_w = 0;

  for ( int i = 0; (i<scan->size()) && (forces_number<force_vectors.size()); ++i)
  {
    VPoint& point = (*scan)[i];
    //point coordinate in map (world) system
    force_points[forces_number] = transform(tf::Vector3(point.x, point.y, point.z));
    //force vector to nearest point
    force_vectors[forces_number] = findPointVector(force_points[forces_number]);
    if ( force_vectors[forces_number].w() > (EMPTY_CELL-1) ) //point is out of spheres - no nearest point? or out of the map
      continue;
    double w = force_vectors[forces_number].w();
    sum_w += force_vectors[forces_number].length();//w*w;

    force_points[forces_number] -= transform.getOrigin();
    forces_number++;

  }
  if ( forces_number < scan->size()/3 )
  {
    ROS_WARN_STREAM("no nearest points "<<forces_number);
    return -1;
  }
//  ROS_INFO_STREAM(" accumulated error value = "<<sum_w << " number of points "<<forces_number);

  return  forces_number;
}

void  PCRegistration::calculateTransformVectors()
{
  step_rotation_vector = findEqualRotationVector();
  step_translation_vector = findEqualDisplacement();
}

QTransform PCRegistration::calculateStepRotation(const VPointCloud::Ptr &scan,
                              const QTransform& transform)
{
  QTransform step;
  step.setOrigin( tf::Vector3( tfScalar(0.0),
                               tfScalar(0.0),
                               tfScalar(0.0) ) );
  step.setRotation(tf::Quaternion(step_rotation_vector, step_rotation ));
  return step;
}

QTransform PCRegistration::calculateStepTranslation(const VPointCloud::Ptr &scan,
                              const QTransform& transform)
{
  QTransform step;
  tf::Vector3 translation(step_translation_vector);
  translation *= (step_translation / step_translation_vector.length());
  step.setOrigin(translation);
  step.setRotation( tf::Quaternion( tfScalar(0.0),
                                    tfScalar(0.0),
                                    tfScalar(0.0),
                                    tfScalar(1.0)) );
  return step;
}

QTransform PCRegistration::calculateStep(const VPointCloud::Ptr &scan,
                              const QTransform& transform)
{
  QTransform step;
  tf::Vector3 translation(step_translation_vector);
  translation *= (step_translation / step_translation_vector.length());
  step.setOrigin(translation);
  step.setRotation(tf::Quaternion(step_rotation_vector, step_rotation));
  return step;
}

void PCRegistration::sumTransform(const QTransform& step)
{
  rotation_sum = rotation_sum*step.getRotation();
  last_rotations.push_back(step.getRotation());

  if ( rotation_sum_num == max_sum_num )
  {
    rotation_sum = last_rotations.front().inverse()*rotation_sum;
    last_rotations.pop_front();
  }
  else
  {
    rotation_sum_num++;
  }

  translation_sum += step.getOrigin();
  last_translations.push_back(step.getOrigin());
  if ( translation_sum_num == max_sum_num )
  {
    translation_sum -= last_translations.front();
    last_translations.pop_front();
  }
  else
  {
    translation_sum_num++;
  }
}

void PCRegistration::applyStepTransform( QTransform& transform,
                                        const QTransform& step)
{
  transform *= step;
#ifdef DEBUG_STEP
  tf::Vector3 v = step.getOrigin();
  tf::Vector3 w = step.getRotor();
  out_v<<v.x()<<" "<<v.y()<<" "<<v.z()<<std::endl;
  out_w<<w.x()<<" "<<w.y()<<" "<<w.z()<<std::endl;
#endif
  sumTransform(step);

}

bool PCRegistration::testStepTransform(const VPointCloud::Ptr &scan,
                           const QTransform& transform,
                           const QTransform& step,
                           double& value)
{
  QTransform test = transform * step;
  double new_value = calculateForces(scan, test);
  if ( value*0.99 < new_value )
  {
    value = new_value;
    return true;
  }
  return false;
}

tf::tfVector4 PCRegistration::findPointVector(double x, double y, double z)
{
  cell_t cell;

  if ( !map.getVal(x,  y,  z, cell))
    return tf::tfVector4(0, 0, 0, EMPTY_CELL);
  return tf::tfVector4(cell.x, cell.y, cell.z, cell.range);
}

tf::tfVector4 PCRegistration::findPointVector(tf::Vector3& point)
{
  cell_t cell;

  if ( !map.getVal(point.x(),  point.y(),  point.z(), cell))
    return tf::tfVector4(0, 0, 0, EMPTY_CELL);
  return tf::tfVector4(cell.x, cell.y, cell.z, cell.range);
}

tf::Vector3 PCRegistration::findEqualDisplacement()
{
  tf::Vector3 displacement(0,0,0);

  for ( int i = 0; i < forces_number; i++ )
  {
    displacement += force_vectors[i];
  }
  tf::Vector3 res = displacement;
  tfScalar res_len =  res.length();
//  ROS_INFO_STREAM("force = "<<res_len<<" ( "<<displacement.x()<<" "
//                                      <<displacement.y()<<" "
//                                      <<displacement.z()<<" )");
  res_len /= forces_number;
  if ( res_len > step_translation ) // 1mm
  {
    res *= step_translation/res_len;
  }
  return res;
}

tf::Quaternion PCRegistration::findEqualRotation()
{
  tf::Vector3 rotor_sum(0,0,0);

  for ( int i = 0; i < forces_number; i++)
  {
    //each vector adds an rotor vector proportional to sin of angle to rotate
    //arm to end of vector
    tf::Vector3 end_arm = force_points[i] + force_vectors[i];
    tf::Vector3 rotor = force_points[i].cross(end_arm)/
                  end_arm.length()/force_points[i].length();
    rotor_sum += rotor ;
  }
  double angle = rotor_sum.length();
//  ROS_INFO_STREAM("rotor = "<<angle<< " ( "<<rotor_sum.x()<<
//                  " "<<rotor_sum.y()<<
//                  " "<<rotor_sum.z() << " )");
  angle /= forces_number;
  //TODO make const
  if ( angle < 0.0000001 )
    return tf::Quaternion::getIdentity();
  if ( angle > step_rotation )
    angle = step_rotation;
  return tf::Quaternion(rotor_sum, angle);
}

tf::Vector3 PCRegistration::findEqualRotationVector()
{
    tf::Vector3 rotor_sum(0,0,0);

    for ( int i = 0; i < forces_number; i++)
    {
      //each vector adds an rotor vector proportional to sin of angle to rotate
      //arm to end of vector
      tf::Vector3 end_arm = force_points[i] + force_vectors[i];
      tf::Vector3 rotor = force_points[i].cross(end_arm)/ end_arm.length()
                                          /force_points[i].length();
      rotor_sum += rotor ;
    }
//    ROS_INFO_STREAM("rotor = "<<rotor_sum.length()<<
//                    " ("<<rotor_sum.x()<<" "<<
//                    rotor_sum.y()<<" "<<
//                    rotor_sum.z()<<" )");
    double angle = rotor_sum.length();
    //angle > 0
    if ( angle > step_rotation )
        rotor_sum /= (angle/step_rotation);
    return rotor_sum;
}

void PCRegistration::setPointCloud(const VPointCloud::Ptr &scan)
{
  size_t map_points(0);
  for(size_t i = 0; i< scan->size(); ++i)
  {
    VPoint& laser = (*scan)[i];
    if ( addPoint(laser.x, laser.y, laser.z) )
      map_points++;
  }
  ROS_DEBUG_STREAM("set to map "<<map_points<<" points");
}

void PCRegistration::setPointCloudFast(const VPointCloud::Ptr &scan)
{
  for (size_t i = 0; i < scan->size(); ++i)
  {
    VPoint& laser = (*scan)[i];
    addPointFast(laser.x, laser.y, laser.z);
  }
}

void PCRegistration::clearPointCloud(const VPointCloud::Ptr &scan)
{
  size_t map_points(0);
  for(size_t i = 0; i< scan->size(); ++i)
  {
    VPoint& laser = (*scan)[i];
    if ( setPoint(laser.x, laser.y, laser.z) )
      map_points++;
  }
  ROS_DEBUG_STREAM("clear to map "<<map_points<<" points");
}

void PCRegistration::clearPointCloudFast(const VPointCloud::Ptr &scan)
{
  for (size_t i = 0; i < scan->size(); ++i)
  {
    VPoint& laser = (*scan)[i];
    setPointFast(laser.x, laser.y, laser.z);
  }
}

void PCRegistration::setPointCloud(const VPointCloud::Ptr& scan,
                                   const QTransform& transform)
{
  size_t map_points(0);
  for(size_t i = 0; i< scan->size(); ++i)
  {
    VPoint& laser = (*scan)[i];
    tf::Vector3 point = transform(tf::Vector3(laser.x, laser.y, laser.z));
    if ( addPoint(point) )
      map_points++;
  }
  ROS_DEBUG_STREAM("set to map "<<map_points<<" points");
}

void PCRegistration::decreaseStepbyAngle()
{
  if ( old_step_translation.angle(step_translation_vector) > 1*M_PI/2 ) //angle = acos > 0
  {
    if (step_translation > MIN_DISPLACEMENT)
      step_translation /= 2;
    ROS_DEBUG_STREAM("decrease translation "<<step_translation);
  }
  if ( old_step_rotation.angle(step_rotation_vector) > 1*M_PI/2 )
  {
    if (step_rotation > MIN_ROTATION)
      step_rotation /= 2;
    ROS_DEBUG_STREAM("decrease rotation "<<step_rotation);
  }
  old_step_translation = step_translation_vector;
  old_step_rotation = step_rotation_vector;

}

bool PCRegistration::decreaseTranslation()
{
  if (step_translation < MIN_DISPLACEMENT)
  {
    return false;
  }
  step_translation /= 2;
  //clear history
  translation_sum_num = 0;
  translation_sum.setZero();
  last_translations.clear();
//  ROS_INFO_STREAM("decrease translation "<<step_translation);
  return true;
}

bool PCRegistration::decreaseRotation()
{
  if ( step_rotation < MIN_ROTATION )
  {
    return false;
  }
  //reduce step
  step_rotation /= 2;
  //clear history
  rotation_sum_num = 0;
  rotation_sum.setValue(0,0,0,1);
  last_rotations.clear();
//  ROS_INFO_STREAM("decrease rotation "<<step_rotation);
  return true;
}


void PCRegistration::decreaseStepbySum()
{
  double rotation_sum_value = rotation_sum.getAngle();
  double translation_sum_value = translation_sum.length();
//  ROS_INFO_STREAM("translation sum of "<<translation_sum_num<<" "<<
//                  translation_sum_value<<" ("
//                  <<translation_sum.x()<<" "
//                  <<translation_sum.y()<<" "
//                  <<translation_sum.z()<<")");
//  ROS_INFO_STREAM("rotation sum of "<<rotation_sum_num<<" "<<rotation_sum_value);
  if ( rotation_sum_num >= max_sum_num)
  {
    if ( rotation_sum_value < 2*step_rotation)
    {
      decreaseRotation();
    }
  }
  if ( translation_sum_num >= max_sum_num )
  {
    if ( translation_sum_value < 2*step_translation )
    {
      decreaseTranslation();
    }
  }
}

void print_transform(const QTransform& t, const std::string& msg="")
{
  const tf::Vector3& Vstep_t = t.getOrigin();
  ROS_DEBUG_STREAM(
      msg<<"\t: translation "<<Vstep_t.length()<< " ( "
      <<Vstep_t.x()<<" "<<Vstep_t.y()<<" "<<Vstep_t.z() <<" )");
  const tf::Vector3& Vstep_r = t.getRotation().getAxis();
  ROS_DEBUG_STREAM(
      msg<<"\t: rotation "<<t.getRotation().getAngle()
      << " ( "<<Vstep_r.x()<<" "<<Vstep_r.y()<<" "<<Vstep_r.z() <<" )");
}

int PCRegistration::findTransform(const VPointCloud::Ptr& scan, QTransform& transform)
{
  force_vectors.resize(scan->size());
  force_points.resize(force_vectors.size());
  step_rotation = MAX_ROTATION;
  step_translation = MAX_DISPLACEMENT;
  weight_sum = 0;
  QTransform step;
  QTransform old_step;
  QTransform sum_step;
  sum_step.setIdentity();
  translation_sum_num = 0;
  rotation_sum_num = 0;
  translation_sum.setZero();
  rotation_sum.setValue(0.0, 0.0, 0.0, 1.0);
#ifdef DEBUG_STEP
  out_v.open("step_v.log", std::ofstream::trunc);
  out_w.open("step_w.log", std::ofstream::trunc);
#endif


  step.setIdentity();
  double closeness = calculateForces(scan, transform);

  old_step_translation = step_translation_vector;
  old_step_rotation = step_rotation_vector;
  int count = 0;

  do
  {
    calculateTransformVectors();
    step = calculateStep(scan, transform);
    if ( testStepTransform(scan, transform, step, closeness ))
    {
//      ROS_INFO_STREAM(count<<": step =");
//      print_transform(step);
      applyStepTransform(transform, step);
      sum_step*=step;
    }
    else //wrong step
    {
      ROS_DEBUG_STREAM("wrong step");
      bool dec_rot = decreaseRotation();
      bool dec_trans = decreaseTranslation();
      if ( dec_rot || dec_trans )
        continue;
      else
      {
        count = -1;
        break;
      }
    }

//    decreaseStepbyAngle();
    decreaseStepbySum();

    count++;
  }while( count < max_iterations_number
      && ( (step_translation > MIN_DISPLACEMENT)
          || (step_rotation > MIN_ROTATION) )

           );
#ifdef DEBUG_STEP
  out_v.close();
  out_w.close();
#endif
 // assert(count < 200);
  ROS_DEBUG_STREAM("iterations number = "<<count<<" forces_number = "<<forces_number);
  ROS_DEBUG_STREAM("sum_step =");
  print_transform(sum_step);
  return count;

}

PCRegistration::PCRegistration(double origX, double origY, double origZ,
                 double sizeX, double sizeY, double sizeZ,
                 double dx, double radius,
                 size_t max_iter ,
                 double max_dis , //m
                 double max_rot , //rad
                 double min_dis ,
                 double min_rot ):
                   map(origX, origY, origZ, sizeX, sizeY, sizeZ, dx, cell_t(EMPTY_CELL)),
                   point_field_radius(radius),
                   max_iterations_number(max_iter),
                   //point_sphere(dx, radius, InvQuadric(radius)),
                   point_sphere(dx, radius, Gaussian(50.0, 0.5)),
                   clear_sphere(point_sphere),
                   MAX_DISPLACEMENT(max_dis), //m
                   MAX_ROTATION (max_rot), //rad
                   MIN_DISPLACEMENT(min_dis),
                   MIN_ROTATION (min_rot),
                   max_sum_num(4)
  {
    clear_sphere.fill( map.getEmptyCell());
  }

} /* namespace registration */
