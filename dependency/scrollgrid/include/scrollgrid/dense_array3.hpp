#ifndef DENSE_ARRAY3_HPP_JEO7CAXQ
#define DENSE_ARRAY3_HPP_JEO7CAXQ

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
 * A dense 3D array.
 * Maps ijk to an CellT.
 * No notion of origin, scrolling etc.
 * The *Grid3 classes handle that.
 * TODO maybe use boost::shared_array,
 * or std::shared_array if we move to C++11
 *
 * Note: when using this to manage memory it's probably not
 * thread-safe.
 */
template<class CellT>
class DenseArray3 {
public:
  typedef CellT CellType;
  typedef CellT * ArrayType;
  typedef CellT * iterator;
  typedef const CellT * const_iterator;

  typedef boost::shared_ptr<DenseArray3> Ptr;
  typedef boost::shared_ptr<const DenseArray3> ConstPtr;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

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

  DenseArray3() :
      dimension_(0, 0, 0),
      num_cells_(0),
      strides_(0, 0, 0),
      grid_(NULL),
      begin_(NULL),
      end_(NULL),
      owns_memory_(false)
  { }

  /**
   * Create a new storage assuming ownership of the memory.
   */
  DenseArray3(const Vec3Ix& dimension) :
      dimension_(dimension),
      num_cells_(dimension.prod()),
      strides_(dimension.tail<2>().prod(), dimension[2], 1),
      grid_(new CellT[num_cells_]()),
      begin_(&grid_[0]),
      end_(&grid_[0]+num_cells_),
      owns_memory_(true)
  { }

  /**
   * Create new storage wrapping an external chunk of memory.
   * Does not assume ownership of the memory, i.e. you must manage it yourself.
   */
  DenseArray3(const Vec3Ix& dimension, ArrayType grid_data) :
      dimension_(dimension),
      num_cells_(dimension.prod()),
      strides_(dimension.tail<2>().prod(), dimension[2], 1),
      grid_(grid_data),
      begin_(&grid_[0]),
      end_(&grid_[0]+num_cells_),
      owns_memory_(false)
  { }

  /**
   * Warning:
   * This only creates a WRAPPER of the memory in other, not a copy.
   * But there is no fancy memory management. Be careful!
   */
  DenseArray3(const DenseArray3& other) :
      dimension_(other.dimension_),
      num_cells_(other.num_cells_),
      strides_(other.strides_),
      grid_(other.grid_),
      begin_(other.begin_),
      end_(other.end_),
      owns_memory_(false)
  {
  }

  /**
   * Warning:
   * This only creates a WRAPPER of the memory in other, not a copy.
   * But there is no fancy memory management. Be careful!
   */
  DenseArray3& operator=(const DenseArray3& other) {
    if (this==&other) { return *this; }
    dimension_ = other.dimension_;
    num_cells_ = other.num_cells_;
    strides_ = other.strides_;
    grid_ = other.grid_;
    begin_ = other.begin_;
    end_ = other.end_;
    owns_memory_ = false;
  }

  /**
   * Allocates storage with same size as other and copies contents from other.
   */
  void ResetAndCopyFrom(const DenseArray3& other) {
    this->reset(other.dimension());
    std::copy(other.begin(), other.end(), this->begin());
  }

  /**
   * Copies contents from other.
   * WARNING: This assumes the current storage has the same dimensions
   * as other. If this is not true, this will crash.
   */
  void CopyFrom(const DenseArray3& other) {
    std::copy(other.begin(), other.end(), this->begin());
  }

  virtual ~DenseArray3() {
    if (owns_memory_ && grid_) { delete[] grid_; grid_ = NULL; }
  }

  /**
   * Free memory (if we have ownership) and reallocate.
   */
  void reset(const Vec3Ix& dimension) {
    dimension_ = dimension;
    num_cells_ = dimension.prod();
    // TODO configurable
    strides_ = Vec3Ix(dimension.tail<2>().prod(), dimension[2], 1);
    if (owns_memory_ && grid_) { delete[] grid_; }
    // note new T[n]() != new T[n];
    // the former initializes to zero
    grid_ = new CellT[num_cells_]();
    begin_ = &(grid_[0]);
    end_ = &(grid_[0])+num_cells_;
  }

  /**
   * Free memory (if we have ownership) and wrap memory in grid_data.
   * Does NOT assume ownership.
   */
  void reset(const Vec3Ix& dimension, ArrayType grid_data) {
    if (owns_memory_ && grid_) { delete[] grid_; }
    dimension_ = dimension;
    num_cells_ = dimension.prod();
    // TODO configurable
    strides_ = Vec3Ix(dimension.tail<2>().prod(), dimension[2], 1);
    owns_memory_ = false;
    grid_ = grid_data;
    begin_ = &(grid_[0]);
    end_ = &(grid_[0])+num_cells_;
  }

  size_t allocated_bytes() {
    return sizeof(CellT)*num_cells_;
  }


public:

  void fill(const CellT& val) {
    std::fill(this->begin(), this->end(), val);
  }

public:
  iterator begin() const { return begin_; }
  iterator end() const { return end_; }

public:

  /**
   * NOTE be careful when using these!
   * They do no take into account any sort of wrapping
   * coming from scrollgrid.
   * CAREFUL.
   */
  grid_ix_t local_grid_to_mem(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    return this->local_grid_to_mem(Vec3Ix(i, j, k));
  }

  grid_ix_t local_grid_to_mem(const Vec3Ix& grid_ix) const {
    return strides_.dot(grid_ix);
  }

  Vec3Ix local_mem_to_grid(grid_ix_t mem_ix) const {
    grid_ix_t i = mem_ix/strides_[0];
    mem_ix -= i*strides_[0];
    grid_ix_t j = mem_ix/strides_[1];
    mem_ix -= j*strides_[1];
    grid_ix_t k = mem_ix;
    return Vec3Ix(i, j, k);
  }

public:

  CellType& get(grid_ix_t i, grid_ix_t j, grid_ix_t k) {
    grid_ix_t mem_ix = this->local_grid_to_mem(i, j, k);
    return grid_[mem_ix];
  }

  const CellType& get(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    grid_ix_t mem_ix = this->local_grid_to_mem(i, j, k);
    return grid_[mem_ix];
  }

  CellType& get(const Vec3Ix& grid_ix) {
    grid_ix_t mem_ix = this->local_grid_to_mem(grid_ix);
    return grid_[mem_ix];
  }

  const CellType& get(const Vec3Ix& grid_ix) const {
    grid_ix_t mem_ix = this->local_grid_to_mem(grid_ix);
    return grid_[mem_ix];
  }

public:

  /**
   * Bound check with ROS_ASSERT
   * Not 'really' safe
   */
  CellType& get_safe(grid_ix_t mem_ix) {
    ROS_ASSERT(mem_ix >= 0 && mem_ix < num_cells_);
    return grid_[mem_ix];
  }

  /**
   * Bound check with ROS_ASSERT
   * Not 'really' safe
   */
  const CellType& get_safe(grid_ix_t mem_ix) const {
    ROS_ASSERT(mem_ix >= 0 && mem_ix < num_cells_);
    return grid_[mem_ix];
  }

public:

  /**
   * No bound check
   */
  CellType& operator[](grid_ix_t mem_ix) {
    return grid_[mem_ix];
  }

  /**
   * No bound check
   */
  const CellType& operator[](grid_ix_t mem_ix) const {
    return grid_[mem_ix];
  }

public:
  // properties
  grid_ix_t dim_i() const { return dimension_[0]; }
  grid_ix_t dim_j() const { return dimension_[1]; }
  grid_ix_t dim_k() const { return dimension_[2]; }
  Vec3Ix dimension() const { return dimension_; }
  grid_ix_t num_cells() const { return num_cells_; }

private:
  // number of grid cells along each axis
  Vec3Ix dimension_;

  // number of cells
  grid_ix_t num_cells_;

  // grid strides to translate from linear to 3D layout.
  // C-ordering, ie x slowest, z fastest.
  Vec3Ix strides_;

  ArrayType grid_;
  iterator begin_, end_;

  // does this object own the grid_ mem
  bool owns_memory_;

};

} /* ca */

#endif /* end of include guard: DENSE_ARRAY3_HPP_JEO7CAXQ */
