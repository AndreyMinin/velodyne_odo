/*
 * QTransform.h
 *
 *  Created on: 12.01.2015
 *      Author: user
 */

#ifndef QTRANSFORM_H_
#define QTRANSFORM_H_

#include <tf/LinearMath/Transform.h>

namespace registration
{
//special class for
class QTransform
{
protected:
  tf::Quaternion m_rotation;
  tf::Vector3 m_origin;
public:
  QTransform(){}
  QTransform(const tf::Transform& transf):
    m_rotation(transf.getRotation()), m_origin(transf.getOrigin()){}
  QTransform(const tf::Quaternion& q, const tf::Vector3& v):
    m_rotation(q), m_origin(v){}
  QTransform(const tf::Vector3& w, const tf::Vector3& v):
    m_origin(v)
  {
    tfScalar len = w.length();
    if ( len < 0.00001 )
      m_rotation.setValue(0.0, 0.0, 0.0, 1.0);
    else
      m_rotation.setRotation(w, len);
  }
  QTransform(const QTransform& other):
    m_rotation(other.m_rotation),m_origin(other.m_origin){}
  QTransform& operator=(const QTransform& other)
  {
    m_rotation = other.m_rotation;
    m_origin = other.m_origin;
    return *this;
  }
  void mult(const QTransform& t1, const QTransform& t2)
  {
    m_rotation = t1.m_rotation*t2.m_rotation;
    m_rotation.normalize();
    m_origin = t1(t2.m_origin);
  }
  tf::Vector3 operator()(const tf::Vector3& x) const
  {
    return tf::quatRotate(m_rotation, x) + m_origin;
  }
  tf::Vector3 operator*(const tf::Vector3& x) const
  {
    return (*this)(x);
  }
  tf::Transform transform()
  {
    return tf::Transform(m_rotation, m_origin);
  }
  QTransform& operator*=(const QTransform& t)
  {
    m_origin +=  tf::quatRotate(m_rotation, t.m_origin);
    m_rotation *= t.m_rotation;
    m_rotation.normalize();
    return *this;
  }
  //
  QTransform& operator*=(tfScalar factor)
  {
    m_origin *=  factor;
    m_rotation.setRotation(m_rotation.getAxis(), m_rotation.getAngle()*factor);
    return *this;
  }


  void setIdentity()
  {
    m_origin.setValue(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0));
    m_rotation.setValue(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0), tfScalar(1.0));
  }
  QTransform inverse() const
  {
    tf::Quaternion inv = m_rotation.inverse();
    return QTransform(inv, tf::quatRotate(inv, -m_origin));
  }
  QTransform operator*(const QTransform& t) const
  {
    return QTransform((m_rotation*t.m_rotation).normalized(),
                    (*this)(t.m_origin));
  }
  static const QTransform&       getIdentity()
  {
    static const QTransform identityTransform(tf::Quaternion(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0), tfScalar(1.0)),
                                                  tf::Vector3(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0)));
    return identityTransform;
  }
  tf::Quaternion&       getRotation()          { return m_rotation; }
  const  tf::Quaternion& getRotation()    const { return m_rotation; }
  tf::Vector3&         getOrigin()         { return m_origin; }
  const tf::Vector3&   getOrigin()   const { return m_origin; }

  tf::Vector3 getRotor() const
  {
    return  m_rotation.getAxis()*m_rotation.getAngle();
  }
  void setRotation(const tf::Quaternion& q)
  {
     m_rotation = q;
  }

  void setOrigin(const tf::Vector3& v)
  {
    m_origin = v;
  }

};

} /* namespace registration */
#endif /* QTRANSFORM_H_ */
