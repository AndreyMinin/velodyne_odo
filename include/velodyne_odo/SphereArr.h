/*
 * SphereArr.h
 *
 *  Created on: 13.11.2014
 *      Author: user
 *      Sphere Arr contains all index biases and values to fill in sphere
 */

#ifndef SPHEREARR_H_
#define SPHEREARR_H_

#include <vector>
#include <stddef.h>
#include <math.h>

namespace registration
{
template <class T, class Val>
class SphereArr
{
public :
  struct Bias
  {
    int i;
    int j;
    int k;
    Val val;
    Bias( int _i, int _j, int _k, Val _val):i(_i), j(_j),k(_k),val(_val){}
  };
protected:

  std::vector<Bias> Arr;
  int dim;
  T _dx;
public:
  typedef typename std::vector<Bias>::const_iterator const_iterator;
  const_iterator begin() const {return Arr.begin();}
  const_iterator end() const {return Arr.end(); }
  size_t size() const { return Arr.size() ; }
  int dimension()const {return dim;}
  int radius()const {return dimension(); }
  T dX() const { return _dx; }
  T maximum() const {return _dx*dim ;}
  void fill(const Val& val)
  {
    for ( typename std::vector<Bias>::iterator it = Arr.begin();
        it != Arr.end(); it++)
    {
      it->val = val;
    }
  }
  template <class Dep>
  SphereArr(T dx, T max, Dep dep):_dx(dx)
  {
    dim = max / dx;
    int full_dim = 2*dim + 1;
    max *= max;
    Arr.reserve(full_dim*full_dim*full_dim);
    for ( int i = -dim; i<= dim; i++)
    {
      T x = (dx*i);
      x *=x;
      for ( int j = -dim; j<= dim; j++)
      {
        T y = (dx*j);
        y *= y;
        T xy = x + y;
        for ( int k = -dim; k<= dim; k++)
        {
          T z = (dx*k);
          z *=z;
          //distance from sphere center
          T range = xy + z;
          if ( range <= max )
          {
            Arr.push_back( Bias(i, j, k, dep(i, j, k, sqrt(range))) ) ;
          }// for k
        }//for j
      }//for i
    }
  }
  SphereArr(const SphereArr& sph):
    Arr(sph.begin(), sph.end()),
    dim(sph.dimension()),
    _dx(sph.dX()){}
};

} /* namespace registration */
#endif /* SPHEREARR_H_ */
