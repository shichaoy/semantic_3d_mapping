#ifndef DENSE_ARRAY2_HPP_ZJGDW1JR
#define DENSE_ARRAY2_HPP_ZJGDW1JR

#include <math.h>
#include <stdint.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/console.h>

#include <geom_cast/geom_cast.hpp>
#include <pcl_util/point_types.hpp>

#include "scrollgrid/grid_types.hpp"

namespace ca
{

/**
 * A dense 2D array.
 * Maps ij to an CellT.
 * No notion of origin, scrolling etc.
 * The *Grid2 classes handle that.
 *
 */
template<class CellT>
class DenseArray2 {
public:
  typedef CellT CellType;
  typedef CellT * ArrayType;
  typedef CellT * iterator;
  typedef const CellT * const_iterator;

  typedef boost::shared_ptr<DenseArray2> Ptr;
  typedef boost::shared_ptr<const DenseArray2> ConstPtr;

public:
  // TODO perhaps stride/memory layout should
  // be further configurable.
  // TODO maybe *grid3 should just map to ijk and then
  // this class will map to linear memory address.
  // problem: then it needs scrolling info
  // I guess we can separate
  // - world_to_grid (infinite grid)
  // - grid_to_storage (linear mem or hash key)
  // - storage (dense array or sparse table)
  //

  DenseArray2() :
      dimension_(0, 0),
      num_cells_(0),
      strides_(0, 0),
      grid_(NULL),
      begin_(NULL),
      end_(NULL)
  { }

  DenseArray2(const Vec2Ix& dimension) :
      dimension_(dimension),
      num_cells_(dimension.prod()),
      strides_(dimension[1], 1),
      grid_(new CellT[num_cells_]),
      begin_(&grid_[0]),
      end_(&grid_[0]+num_cells_)
  { }

  virtual ~DenseArray2() {
    if (grid_) { delete[] grid_; }
  }

  void reset(const ca::Vec2Ix& dimension) {
    if (grid_) { delete[] grid_; }
    dimension_ = dimension;
    num_cells_ = dimension.prod();
    strides_ = ca::Vec2Ix(dimension[1], 1);
    grid_ = new CellT[num_cells_]();
    begin_ = &grid_[0];
    end_ = &grid_[0]+num_cells_;
  }

public:

  /**
   * NOTE be careful when using these!
   * They do no take into account any sort of wrapping
   * coming from scrollgrid.
   */
  grid_ix_t grid_to_mem(grid_ix_t i, grid_ix_t j) const {
    return this->grid_to_mem(Vec2Ix(i, j));
  }

  grid_ix_t grid_to_mem(const Vec2Ix& grid_ix) const {
    return strides_.dot(grid_ix);
  }

  /**
   * Note: no bound checking.
   */
  CellType& get(grid_ix_t i, grid_ix_t j) {
    grid_ix_t mem_ix = this->grid_to_mem(i, j);
    return grid_[mem_ix];
  }

  const CellType& get(grid_ix_t i, grid_ix_t j) const {
    grid_ix_t mem_ix = this->grid_to_mem(i, j);
    return grid_[mem_ix];
  }

  CellType& get(const Vec2Ix& grid_ix) {
    grid_ix_t mem_ix = this->grid_to_mem(grid_ix);
    return grid_[mem_ix];
  }

  const CellType& get(const Vec2Ix& grid_ix) const {
    grid_ix_t mem_ix = this->grid_to_mem(grid_ix);
    return grid_[mem_ix];
  }

  CellType& get(grid_ix_t mem_ix) {
    return grid_[mem_ix];
  }

  const CellType& get(grid_ix_t mem_ix) const {
    return grid_[mem_ix];
  }

  CellType& operator[](mem_ix_t mem_ix) {
    return grid_[mem_ix];
  }

  const CellType& operator[](mem_ix_t mem_ix) const {
    return grid_[mem_ix];
  }



public:
  // properties
  grid_ix_t dim_i() const { return dimension_[0]; }
  grid_ix_t dim_j() const { return dimension_[1]; }
  Vec2Ix dimension() const { return dimension_; }
  grid_ix_t num_cells() const { return num_cells_; }

private:
  DenseArray2(const DenseArray2& other);
  DenseArray2& operator=(const DenseArray2& other);

private:
  // number of grid cells along each axis
  Vec2Ix dimension_;

  // number of cells
  grid_ix_t num_cells_;

  // grid strides to translate from linear to 3D layout.
  // C-ordering, ie x slowest, z fastest.
  Vec2Ix strides_;

  ArrayType grid_;
  iterator begin_, end_;
};

} /* ca */

#endif /* end of include guard: DENSE_ARRAY2_HPP_ZJGDW1JR */
