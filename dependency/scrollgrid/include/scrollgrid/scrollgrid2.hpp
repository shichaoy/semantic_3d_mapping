#ifndef SCROLLGRID2_HPP_YPBBYE5Q
#define SCROLLGRID2_HPP_YPBBYE5Q

#include <math.h>
#include <stdint.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_util/point_types.hpp>
#include <geom_cast/geom_cast.hpp>

#include "scrollgrid/mod_wrap.hpp"
#include "scrollgrid/grid_types.hpp"
#include "scrollgrid/box.hpp"

namespace ca
{

template<class Scalar>
class ScrollGrid2 {
public:
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2;

  typedef boost::shared_ptr<ScrollGrid2> Ptr;
  typedef boost::shared_ptr<const ScrollGrid2> ConstPtr;

public:
  ScrollGrid2() :
      box_(),
      origin_(0, 0),
      min_world_corner_ij_(0, 0),
      dimension_(0, 0),
      num_cells_(0),
      strides_(0, 0),
      scroll_offset_(0, 0),
      last_ij_(0, 0),
      wrap_ij_min_(0, 0),
      wrap_ij_max_(0, 0),
      resolution_(0)
  { }

  ScrollGrid2(const Vec2& center,
              const Vec2Ix& dimension,
              Scalar resolution,
              bool x_fastest=false) :
      box_(center-(dimension.cast<Scalar>()*resolution)/2,
           center+(dimension.cast<Scalar>()*resolution)/2),
      origin_(center-box_.radius()),
      dimension_(dimension),
      num_cells_(dimension.prod()),
      strides_(dimension[1], 1),
      scroll_offset_(0, 0, 0),
      last_ij_(scroll_offset_ + dimension_),
      resolution_(resolution)
  {

    Vec2 m;
    m[0] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    m[1] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    min_world_corner_ij_ = this->world_to_grid(m);

    if (x_fastest) {
      strides_ = Vec2Ix(1, dimension[0]);
    }

    this->update_wrap_ij();
  }

  virtual ~ScrollGrid2() { }

  ScrollGrid2(const ScrollGrid2& other) :
      box_(other.box_),
      origin_(other.origin_),
      min_world_corner_ij_(other.min_world_corner_ij_),
      dimension_(other.dimension_),
      num_cells_(other.num_cells_),
      strides_(other.strides_),
      scroll_offset_(other.scroll_offset_),
      last_ij_(other.last_ij_),
      wrap_ij_min_(other.wrap_ij_min_),
      wrap_ij_max_(other.wrap_ij_max_),
      resolution_(other.resolution_)
  {
  }

  ScrollGrid2& operator=(const ScrollGrid2& other) {
    if (*this==other) { return *this; }
    box_ = other.box_;
    origin_ = other.origin_;
    min_world_corner_ij_ = other.min_world_corner_ij_;
    dimension_ = other.dimension_;
    num_cells_ = other.num_cells_;
    strides_ = other.strides_;
    scroll_offset_ = other.scroll_offset_;
    last_ij_ = other.last_ij_;
    wrap_ij_min_ = other.wrap_ij_min_;
    wrap_ij_max_ = other.wrap_ij_max_;
    resolution_ = other.resolution_;
    return *this;
  }

public:

  void reset(const Vec2& center,
             const Vec2Ix& dimension,
             Scalar resolution,
             bool x_fastest=false) {
    box_.set_center(center);
    box_.set_radius((dimension.cast<Scalar>()*resolution)/2);
    origin_ = center - box_.radius();

    dimension_ = dimension;
    num_cells_ = dimension.prod();
    if (x_fastest) {
      strides_ = Vec2Ix(1, dimension[0]);
    } else {
      strides_ = Vec2Ix(dimension[1], 1);
    }
    scroll_offset_.setZero();
    last_ij_ = scroll_offset_ + dimension_;

    this->update_wrap_ij();

    resolution_ = resolution;

    Vec2 m;
    m[0] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    m[1] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    min_world_corner_ij_ = this->world_to_grid(m);

  }

  /**
   * Is inside 3D box containing grid?
   * @param pt point in same frame as center (probably world_view)
   */
  bool is_inside_box(const Vec2& pt) const {
    return box_.contains(pt);
  }

  template<class PointT>
  bool is_inside_box(const PointT& pt) const {
    return box_.contains(ca::point_cast<Vec2>(pt));
  }

  /**
   * is i, j, k inside the grid limits?
   */
  bool is_inside_grid(const Vec2Ix& grid_ix) const {
    return ((grid_ix.array() >= scroll_offset_.array()).all() &&
            (grid_ix.array() < (scroll_offset_+dimension_).array()).all());
  }

  bool is_inside_grid(grid_ix_t i, grid_ix_t j) const {
    return this->is_inside_grid(Vec2Ix(i, j));
  }

  /**
   * scroll grid.
   * updates bounding box and offset_cells.
   * @param offset_cells. how much to scroll. offset_cells is a signed integral.
   *
   */
  void scroll(const Vec2Ix& offset_cells) {
    Vec2Ix new_offset = scroll_offset_ + offset_cells;
    box_.translate((offset_cells.cast<Scalar>()*resolution_));
    scroll_offset_ = new_offset;
    last_ij_ = scroll_offset_ + dimension_;

    this->update_wrap_ij();
  }

  /**
   * get boxes to clear if scrolling by offset_cells.
   * call this *before* scroll().
   * @param clear_i_min min corner of obsolete region in grid
   * @param clear_i_max max corner of obsolete region in grid
   * same for j and k
   * Note that boxes may overlap.
   */
  void get_clear_boxes(const Vec2Ix& offset_cells,
                       Vec2Ix& clear_i_min, Vec2Ix& clear_i_max,
                       Vec2Ix& clear_j_min, Vec2Ix& clear_j_max) {

    Vec2Ix new_offset = scroll_offset_ + offset_cells;

    clear_i_min.setZero();
    clear_j_min.setZero();
    clear_i_max.setZero();
    clear_j_max.setZero();

    // X axis
    if (offset_cells[0] > 0) {
      clear_i_min = scroll_offset_;
      clear_i_max = Vec2Ix(new_offset[0],
                           scroll_offset_[1]+dimension_[1]);
    } else if (offset_cells[0] < 0) {
      clear_i_min = Vec2Ix(scroll_offset_[0]+dimension_[0]+offset_cells[0],
                           scroll_offset_[1]);
      clear_i_max = scroll_offset_ + dimension_;
    }

    // Y axis
    if (offset_cells[1] > 0) {
      clear_j_min = scroll_offset_;
      clear_j_max = Vec2Ix(scroll_offset_[0]+dimension_[0],
                           new_offset[1]);
    } else if (offset_cells[1] < 0) {
      clear_j_min = Vec2Ix(scroll_offset_[0],
                           scroll_offset_[1]+dimension_[1]+offset_cells[1]);
      clear_j_max = scroll_offset_ + dimension_;
    }

  }

  /**
   * Given position in world coordinates, return grid coordinates.
   * (grid coordinates are not wrapped to be inside grid!)
   * Note: does not check if point is inside grid.
   */
  Vec2Ix world_to_grid(const Vec2& xy) const {
    Vec2 tmp = ((xy - origin_).array() - 0.5*resolution_)/resolution_;
    //ROS_INFO_STREAM("tmp = " << tmp);
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

  /**
   * Translate grid indices to an address in linear memory.
   * Does not check if grid_ix is inside current grid box.
   * Assumes C-order, x the slowest and z the fastest.
   */
  mem_ix_t grid_to_mem(const Vec2Ix& grid_ix) const {
    Vec2Ix grid_ix2(ca::mod_wrap(grid_ix[0], dimension_[0]),
                    ca::mod_wrap(grid_ix[1], dimension_[1]));
    return strides_.dot(grid_ix2);
  }

  /**
   * This is faster than grid_to_mem, as it avoids modulo.
   * But it only works if the grid_ix are inside the bounding box.
   * Hopefully branch prediction kicks in
   */
  mem_ix_t grid_to_mem2(const Vec2Ix& grid_ix) const {
    Vec2Ix grid_ix2(grid_ix);
    if (grid_ix2[0] >= wrap_ij_max_[0]) { grid_ix2[0] -= wrap_ij_max_[0]; } else { grid_ix2[0] -= wrap_ij_min_[0]; }
    if (grid_ix2[1] >= wrap_ij_max_[1]) { grid_ix2[1] -= wrap_ij_max_[1]; } else { grid_ix2[1] -= wrap_ij_min_[1]; }
    mem_ix_t mem_ix2 = strides_.dot(grid_ix2);
    return mem_ix2;
  }

  mem_ix_t grid_to_mem(grid_ix_t i, grid_ix_t j) const {
    return this->grid_to_mem(Vec2Ix(i, j));
  }

  mem_ix_t grid_to_mem2(grid_ix_t i, grid_ix_t j) const {
    return grid_to_mem2(Vec2Ix(i, j));
  }

  uint64_t grid_to_hash(const Vec2Ix& grid_ix) const {
    // grid2 should be all positive
    Vec2Ix grid2(grid_ix - min_world_corner_ij_);
    uint64_t hi = static_cast<uint64_t>(grid2[0]);
    uint64_t hj = static_cast<uint64_t>(grid2[1]);
    uint64_t h = (hi << 48) | (hj << 32);
    return h;
  }

  Vec2Ix hash_to_grid(uint64_t hix) const {
    uint64_t hi = (hix & 0xffff000000000000) >> 48;
    uint64_t hj = (hix & 0x0000ffff00000000) >> 32;
    Vec2Ix grid_ix(hi, hj);
    grid_ix += min_world_corner_ij_;
    return grid_ix;
  }

  /**
   * Note that no bound check is performed!
   */
  mem_ix_t world_to_mem(const Vec2& xy) const {
    Vec2 tmp(((xy - origin_).array() - 0.5*resolution_)/resolution_);
    Vec2Ix gix(round(tmp.x()), round(tmp.y()), round(tmp.z()));
    ca::inplace_mod_wrap(gix[0], dimension_[0]);
    ca::inplace_mod_wrap(gix[1], dimension_[1]);
    return strides_.dot(gix);
  }

  mem_ix_t world_to_mem2(const Vec2& xy) const {
    Vec2Ix gix(this->world_to_grid(xy));
    return this->world_to_mem2(gix);
  }

  Vec2Ix mem_to_grid(grid_ix_t mem_ix) const {
    // TODO does this work for x-fastest strides?
    grid_ix_t i = mem_ix/strides_[0];
    mem_ix -= i*strides_[0];
    grid_ix_t j = mem_ix/strides_[1];
    mem_ix -= j*strides_[1];

    // undo wrapping
    grid_ix_t ax = floor(static_cast<Scalar>(scroll_offset_[0])/dimension_[0])*dimension_[0];
    grid_ix_t ay = floor(static_cast<Scalar>(scroll_offset_[1])/dimension_[1])*dimension_[1];

    Vec2Ix fixed_ij;
    fixed_ij[0] = i + ax + (i<(scroll_offset_[0]-ax))*dimension_[0];
    fixed_ij[1] = j + ay + (j<(scroll_offset_[1]-ay))*dimension_[1];

    return fixed_ij;
  }

 public:

  grid_ix_t dim_i() const { return dimension_[0]; }
  grid_ix_t dim_j() const { return dimension_[1]; }
  grid_ix_t first_i() const { return scroll_offset_[0]; }
  grid_ix_t first_j() const { return scroll_offset_[1]; }
  grid_ix_t last_i() const { return last_ij_[0]; }
  grid_ix_t last_j() const { return last_ij_[1]; }
  const Vec2Ix& dimension() const { return dimension_; }
  const Vec2& radius() const { return box_.radius(); }
  const Vec2& origin() const { return origin_; }
  Vec2 min_pt() const { return box_.min_pt(); }
  Vec2 max_pt() const { return box_.max_pt(); }
  const Vec2& center() const { return box_.center(); }
  Scalar resolution() const { return resolution_; }
  const ca::scrollgrid::Box<Scalar, 2>& box() const { return box_; }
  grid_ix_t num_cells() const { return num_cells_; }
  Vec2Ix scroll_offset() const { return scroll_offset_; }

 private:

  void update_wrap_ij() {
    wrap_ij_min_[0] = floor(static_cast<float>(scroll_offset_[0])/dimension_[0])*dimension_[0];
    wrap_ij_min_[1] = floor(static_cast<float>(scroll_offset_[1])/dimension_[1])*dimension_[1];

    wrap_ij_max_[0] = floor(static_cast<float>(scroll_offset_[0]+dimension_[0])/dimension_[0])*dimension_[0];
    wrap_ij_max_[1] = floor(static_cast<float>(scroll_offset_[1]+dimension_[1])/dimension_[1])*dimension_[1];
  }

 private:
  // 2d box enclosing grid. In whatever coordinates were given (probably
  // world_view)
  ca::scrollgrid::Box<Scalar, 2> box_;

  // static origin of the grid coordinate system. does not move when scrolling
  // it's center - box.radius
  Vec2 origin_;

  // minimum world corner in ij. used for hash
  Vec2Ix min_world_corner_ij_;

  // number of grid cells along each axis
  Vec2Ix dimension_;

  // number of cells
  grid_ix_t num_cells_;

  // grid strides to translate from linear to 3D layout.
  // C-ordering, ie x slowest, z fastest.
  Vec2Ix strides_;

  // to keep track of scrolling along z.
  Vec2Ix scroll_offset_;

  // redundant but actually seems to have a performance benefit
  // should always be dimension + offset
  Vec2Ix last_ij_;

  // for grid_to_mem2. the points where the grid crosses modulo boundaries.
  Vec2Ix wrap_ij_min_;
  Vec2Ix wrap_ij_max_;

  // size of grid cells
  Scalar resolution_;

};

typedef ScrollGrid2<float> ScrollGrid2f;
typedef ScrollGrid2<double> ScrollGrid2d;

} /* ca */

#endif /* end of include guard: SCROLLGRID2_HPP_YPBBYE5Q */
