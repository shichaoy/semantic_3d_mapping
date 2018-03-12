#ifndef FIXEDGRID2_HPP_UYCWT1KR
#define FIXEDGRID2_HPP_UYCWT1KR

#include <stdint.h>
#include <math.h>

#include <vector>
#include <algorithm>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/console.h>

#include <geom_cast/geom_cast.hpp>
#include <pcl_util/point_types.hpp>

#include "scrollgrid/grid_types.hpp"
#include "scrollgrid/box.hpp"

namespace ca
{

template<class Scalar>
class FixedGrid2 {
public:
  typedef Scalar ScalarType;
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;

  typedef boost::shared_ptr<FixedGrid2> Ptr;
  typedef boost::shared_ptr<const FixedGrid2> ConstPtr;

public:
  FixedGrid2() :
      box_(),
      origin_(0, 0),
      dimension_(0, 0),
      num_cells_(0),
      strides_(0, 0),
      resolution_(0)
  { }

  FixedGrid2(const Vec2& center,
             const Vec2Ix& dimension,
             Scalar resolution) :
      box_(center-(dimension.cast<Scalar>()*resolution)/2,
           center+(dimension.cast<Scalar>()*resolution)/2),
      origin_(center-box_.radius()),
      dimension_(dimension),
      num_cells_(dimension.prod()),
      strides_(dimension[1], 1),
      resolution_(resolution)
  { }

  FixedGrid2(const FixedGrid2& other) :
      box_(other.box_),
      origin_(other.origin_),
      dimension_(other.dimension_),
      num_cells_(other.num_cells_),
      strides_(other.strides_),
      resolution_(other.resolution_)
  {
  }

  FixedGrid2& operator=(const FixedGrid2& other) {
    if (*this==other) { return *this; }
    box_ = other.box_;
    origin_ = other.origin_;
    dimension_ = other.dimension_;
    num_cells_ = other.num_cells_;
    strides_ = other.strides_;
    resolution_ = other.resolution_;
    return *this;
  }

  virtual ~FixedGrid2() { }

public:
  void reset(const Vec2& center,
             const Vec2Ix& dimension,
             Scalar resolution) {
    box_.set_center(center);
    box_.set_radius((dimension.cast<Scalar>()*resolution)/2);
    origin_ = center - box_.radius();
    dimension_ = dimension;
    num_cells_ = dimension.prod();
    strides_ = Vec2Ix(dimension[1], 1);
    resolution_ = resolution;
  }

  void copy_from(ca::FixedGrid2<Scalar>& other) {
    this->reset(other.center(),
                other.dimension(),
                other.resolution());
  }

  /**
   * Is inside 3D box containing grid?
   * pt is in same frame as center (probably world_view)
   */
  bool is_inside_box(Scalar x, Scalar y) const {
    return box_.contains(Vec2(x, y));
  }

  bool is_inside_box(const Vec2& pt) const {
    return box_.contains(pt);
  }

  template<class PointT>
  bool is_inside_box(const PointT& pt) const {
    return box_.contains(ca::point_cast<Eigen::Vector2d>(pt));
  }

  /**
   * is i, j inside the grid limits?
   */
  bool is_inside_grid(const Vec2Ix& grid_ix) const {
    return ((grid_ix.array() >= 0).all() &&
            (grid_ix.array() < dimension_.array()).all());
  }

  bool is_inside_grid(grid_ix_t i, grid_ix_t j) const {
    return this->is_inside_grid(Vec2Ix(i, j));
  }

  /**
   * Given position in world coordinates, return grid coordinates.
   * Note: does not check if point is inside grid.
   */
  Vec2Ix world_to_grid(const Vec2& xy) const {
    Vec2 tmp = ((xy - origin_).array() - 0.5*resolution_)/resolution_;
    //return tmp.cast<grid_ix_t>();
    return Vec2Ix(round(tmp.x()), round(tmp.y()));
  }

  Vec2Ix world_to_grid(Scalar x, Scalar y) const {
    return this->world_to_grid(Vec2(x, y));
  }

  Vec2 grid_to_world(const Vec2Ix& grid_ix) const {
    Vec2 w((grid_ix.cast<Scalar>()*resolution_ + origin_).array() + 0.5*resolution_);
    return w;
  }

  Vec2 grid_to_world(grid_ix_t i, grid_ix_t j) const {
    return this->grid_to_world(Vec2Ix(i, j));
  }

  mem_ix_t grid_to_mem(grid_ix_t i, grid_ix_t j) const {
    return this->grid_to_mem(Vec2Ix(i, j));
  }

  /**
   * Note that this wraps the z dimension.
   * TODO c-order/f-order config
   */
  mem_ix_t grid_to_mem(const Vec2Ix& grid_ix) const {
    return strides_.dot(grid_ix);
  }

  Vec2Ix mem_to_grid(mem_ix_t mem_ix) const {
    grid_ix_t i = mem_ix/strides_[0];
    mem_ix -= i*strides_[0];
    grid_ix_t j = mem_ix;
    mem_ix -= j;
    return Vec2Ix(i, j);
  }

 public:
  grid_ix_t dim_i() const { return dimension_[0]; }
  grid_ix_t dim_j() const { return dimension_[1]; }
  const Vec2Ix& dimension() const { return dimension_; }
  const Vec2& radius() const { return box_.radius(); }
  const Vec2& origin() const { return origin_; }
  Vec2 min_pt() const { return box_.min_pt(); }
  Vec2 max_pt() const { return box_.max_pt(); }
  const Vec2& center() const { return box_.center(); }
  Scalar resolution() const { return resolution_; }

  grid_ix_t num_cells() const { return num_cells_; }

 private:
  // 3d box enclosing grid. In whatever coordinates were given (probably
  // world_view)
  ca::scrollgrid::Box<Scalar, 2> box_;

  // static origin of the grid coordinate system.
  Vec2 origin_;

  // number of grid cells along each axis
  Vec2Ix dimension_;

  // number of cells
  grid_ix_t num_cells_;

  // grid strides to translate from linear to 3D layout.
  // C-ordering, ie x slowest, z fastest.
  Vec2Ix strides_;

  // size of grid cells
  Scalar resolution_;
};

}

#endif /* end of include guard: FIXEDGRID2_HPP_UYCWT1KR */
