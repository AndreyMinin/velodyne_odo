/*
 * Map3D.h
 *
 *  Created on: 14.11.2014
 *      Author: user
 */

#ifndef MAP3D_H_
#define MAP3D_H_

#include <algorithm>
#include <vector>
#include "SphereArr.h"
#include <string.h>
#include <assert.h>
#include <velodyne_pointcloud/rawdata.h>
#include <iostream>

namespace registration
{
// T is dimension type
//Val is value in grid type
template <class T, class Val>
class Map3D
{
protected:

  //size of cell in every dimension
  T cell_size;
  T map_size_x;
  T map_size_y;
  T map_size_z;

  size_t cell_num_x;
  size_t cell_num_y;
  size_t cell_num_z;

  //X0
  T map_orig_x;
  T map_orig_y;
  T map_orig_z;

  Val empty;
  std::vector<Val> grid;
public:
  int x2Index(T t){ return ((-map_orig_x + t)/cell_size); }
  int y2Index(T t){ return ((-map_orig_y + t)/cell_size); }
  int z2Index(T t){ return ((-map_orig_z + t)/cell_size); }
  T index2x(int i) { return i*cell_size + map_orig_x; }
  T index2y(int j) { return j*cell_size + map_orig_y; }
  T index2z(int k) { return k*cell_size + map_orig_z; }
  bool xInGrid(T t) { T x(-map_orig_x + t); return (x > 0) && (x<map_size_x); }
  bool yInGrid(T t) { T x(-map_orig_y + t); return (x > 0) && (x<map_size_y); }
  bool zInGrid(T t) { T x(-map_orig_z + t); return (x > 0) && (x<map_size_z); }
  bool xIndInGrid(int i) { return i >= 0 && i<cell_num_x; }
  bool yIndInGrid(int i) { return i >= 0 && i<cell_num_y; }
  bool zIndInGrid(int i) { return i >= 0 && i<cell_num_z; }
  T resolution(){ return cell_size; }
  T origin_x(){ return map_orig_x; }
  T origin_y(){ return map_orig_y; }
  T origin_z(){ return map_orig_z; }
  T size_x() { return map_size_x; }
  T size_y() { return map_size_y; }
  T size_z() { return map_size_z; }
  bool pointInGrid(T x, T y, T z)
  {
    return xInGrid(x) && yInGrid(y) && zInGrid(z);
  }
  bool indInGrid(int i, int j, int k)
  {
    return (i>=0) && (i<(int)cell_num_x) && (j>=0) && (j<(int)cell_num_y) && (k>=0) && (k<(int)cell_num_z);
  }
  int indexesToGrid(int i, int j, int k)
  {
    return (k*cell_num_x*cell_num_y + j*cell_num_x + i);
  }
  size_t cellSizeX(){ return cell_num_x; }
  size_t cellSizeY(){ return cell_num_y; }
  size_t cellSizeZ(){ return cell_num_z; }
  Val& operator()(size_t i, size_t j, size_t k)
  {
    size_t index = indexesToGrid(i,j,k);
    assert(index < grid.size() );
    return grid[index];
  }

  bool getVal(T x, T y, T z, Val& val)
  {
    int  i = x2Index(x);
    if (!xIndInGrid(i))
      return false;
    int j = y2Index(y);
    if (!yIndInGrid(j))
      return false;
    int k = z2Index(z);
    if (!zIndInGrid(k))
      return false;
    val = this->operator ()(i, j, k);
    return true;
  }

  Val* getValPtr(T x, T y, T z)
  {
    int  i = x2Index(x);
    if (!xIndInGrid(i))
      return NULL;
    int j = y2Index(y);
    if (!yIndInGrid(j))
      return NULL;
    int k = z2Index(z);
    if (!zIndInGrid(k))
      return NULL;
    return &(grid[ indexesToGrid(i,j,k) ]);
  }

  bool setVal(T x, T y, T z, const Val& val)
  {
    int  i = x2Index(x);
    if (!xIndInGrid(i))
      return false;
    int j = y2Index(y);
    if (!yIndInGrid(j))
      return false;
    int k = z2Index(z);
    if (!zIndInGrid(k))
      return false;
    this->operator ()(i, j, k) = val;
    return true;
  }


  void addSphereFast(int i, int j, int k, const SphereArr<T, Val>& sph)
  {
    for(typename SphereArr<T, Val>::const_iterator it = sph.begin();
        it != sph.end(); it++)
    {
      Val& current_val = operator () (i + it->i, j + it->j, k + it->k);
//      if (it->val < current_val)
//        current_val = it->val;
      current_val += it->val;

    }
  }


  void addSphereFast(T x, T y, T z, const SphereArr<T, Val>& sph)
  {
    addSphereFast(x2Index(x), y2Index(y), z2Index(z), sph);
  }



  bool addSphere(T x, T y, T z, const SphereArr<T, Val>& sph)
  {
    int i = x2Index(x);
    if ( i < - (int)sph.radius() || i > (int)cell_num_x + (int)sph.radius() )
      return false;
    int j = y2Index(y);
    if ( j < - (int)sph.radius() || j > (int)cell_num_y + (int)sph.radius() )
      return false;
    int k = z2Index(z);
    if ( k < - (int)sph.radius() || k > (int)cell_num_z + (int)sph.radius() )
      return false;
    if ( i >= (int)sph.radius() && i < (int)cell_num_x - (int)sph.radius()  &&
        j >=  (int)sph.radius() && j > (int)cell_num_y - (int)sph.radius() &&
        k >=  (int)sph.radius() && k < (int)cell_num_z - (int)sph.radius()  )
    {
      addSphereFast(i, j, k, sph);
      return true;
    }
    //else part of sphere is out of map
    for(typename SphereArr<T, Val>::const_iterator it = sph.begin();
        it != sph.end(); it++)
    {
      int ii = i + it->i;
      int jj = j + it->j;
      int kk = k + it->k;
      if ( indInGrid(ii, jj, kk))
      {
        Val& current_val = operator() (ii, jj, kk);
//        if ( it->val < current_val)
//          current_val = it->val;
        current_val += it->val;
        int s=0;
      }
    }
    return true;
  }

  void setSphereFast(int i, int j, int k, const SphereArr<T, Val>& sph)
   {
     for(typename SphereArr<T, Val>::const_iterator it = sph.begin();
         it != sph.end(); it++)
     {
       Val& current_val = operator () (i + it->i, j + it->j, k + it->k);
       current_val = it->val;
     }
   }

   void setSphereFast(T x, T y, T z, const SphereArr<T, Val>& sph)
   {
     setSphereFast(x2Index(x), y2Index(y), z2Index(z), sph);
   }

   bool setSphere(T x, T y, T z, const SphereArr<T, Val>& sph)
    {
      int i = x2Index(x);
      if ( i < - (int)sph.radius() || i > (int)cell_num_x + (int)sph.radius() )
        return false;
      int j = y2Index(y);
      if ( j < - (int)sph.radius() || j > (int)cell_num_y + (int)sph.radius() )
        return false;
      int k = z2Index(z);
      if ( k < - (int)sph.radius() || k > (int)cell_num_z + (int)sph.radius() )
        return false;
      if ( i >= (int)sph.radius() && i < (int)cell_num_x - (int)sph.radius()  &&
          j >=  (int)sph.radius() && j > (int)cell_num_y - (int)sph.radius() &&
          k >=  (int)sph.radius() && k < (int)cell_num_z - (int)sph.radius()  )
      {
        setSphereFast(i, j, k, sph);
        return true;
      }
      //else part of sphere is out of map
      for(typename SphereArr<T, Val>::const_iterator it = sph.begin();
          it != sph.end(); it++)
      {
        int ii = i + it->i;
        int jj = j + it->j;
        int kk = k + it->k;
        if ( indInGrid(ii, jj, kk))
        {
          Val& current_val = operator() (ii, jj, kk);
  //        if ( it->val < current_val)
  //          current_val = it->val;
          current_val = it->val;
          int s=0;
        }
      }
      return true;
    }

   /*
      template< Val& Val::*op(const Val&) >
  void forSphereFast(int i, int j, int k, const SphereArr<T, Val>& sph)
  {
    for(typename SphereArr<T, Val>::const_iterator it = sph.begin();
        it != sph.end(); it++)
    {
      Val& current_val = operator () (i + it->i, j + it->j, k + it->k);
//      if (it->val < current_val)
//        current_val = it->val;
      //current_val += it->val;
      current_val.op(it->val)
    }
  }

  template <Val& Val::*op(const Val&)>
  void forSphereFast(T x, T y, T z, const SphereArr<T, Val>& sph)
  {
    forSphereFast<Op>(x2Index(x), y2Index(y), z2Index(z), sph);
  }



    */

  void setOrigin(T x, T y, T z)
  {
    map_orig_x = x;
    map_orig_y = y;
    map_orig_z = z;
  }

  void clear()
  {
    grid.clear();
  }

  void zero()
  {
    std::fill(grid.begin(), grid.end(), empty );
  }


  template< class Pred >
  bool test_if(Pred pred)
  {
    typename std::vector<Val>::iterator  it = std::find_if(grid.begin(), grid.end(), pred);
    if ( it != grid.end())
    {
      int ind = std::distance(grid.begin(), it);
      std::cout<<" index = "<<ind<<" grid size = "<<grid.size()<<std::endl;
      int i = ind%cell_num_x;
      int j = ( (ind - i )/cell_num_x )%cell_num_y;
      int k = ((ind - i)/cell_num_x - j)/cell_num_y;
      std::cout<<" i = "<<i<<" j = "<<j<<" k = "<<k<<std::endl;
      std::cout<<" x = "<<index2x(i)<<
              " y = "<<index2y(j)<<
              " z = "<<index2z(k)<<std::endl;
    }
    return it != grid.end();
  }

  const Val& getEmptyCell() const { return empty; }

  Map3D(T x0, T y0, T z0, T sizex, T sizey, T sizez, T dx, const Val& _empty):
    cell_size(dx),
    map_size_x(sizex),
    map_size_y(sizey),
    map_size_z(sizez),
    cell_num_x(ceil(sizex/dx)),
    cell_num_y(ceil(sizey/dx)),
    cell_num_z(ceil(sizez/dx)),
    map_orig_x(x0),
    map_orig_y(y0),
    map_orig_z(z0),
    empty(_empty),
    grid(cell_num_x*cell_num_y*cell_num_z, empty )
  {
    //memset(&grid[0], 0, grid.size()*sizeof(Val));
  }


};

}//namespace


#endif /* MAP3D_H_ */
