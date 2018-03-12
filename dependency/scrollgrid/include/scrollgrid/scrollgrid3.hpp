#ifndef SCROLLGRID3_HPP_I9SAOOSJ
#define SCROLLGRID3_HPP_I9SAOOSJ

#include <math.h>
#include <stdint.h>

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

/**
 * These are empty functors defining the interface for callbacks
 * to clear cells and fix edges.
 * TODO this could also be achieved with templates.
 */
struct ClearCellsFun {
  virtual void operator()(const Vec3Ix& start,
                          const Vec3Ix& finish) const { }
};

struct FixEdgesFun {
  virtual void operator()(grid_ix_t dim,
                          grid_ix_t trailing,
                          grid_ix_t leading) const { }
};

//typedef boost::function< void (grid_ix_t dim, grid_ix_t trailing, grid_ix_t leading) > FixEdgesFun;
/**
 * "world_xyz" xyz world frame
 * "grid_xyz" xyz grid frame (shifted relative to world_xyz by origin)
 * "grid_ijk" scaled and discretized grid coordinates:
 *     i = floor( (x-origin_x-0.5)/resolution )
 * "local_ijk" grid_ijk shifted by scrolling and limited/wrapped to local extent
 *     li = (i - scroll_offset_i) modulo (dim_i)
 * "mem_ix" index into flat storage from local_ijk.
 *     mem_ix = local_ijk.dot(strides)
 * "hash_ix": bit-packed version of grid_ijk? TODO
 *
 */

// TODO start using vec4 instead for SSE optimization

template<class Scalar>
class ScrollGrid3 {
public:
  // TODO what if used vec4 instead for SSE optimizations?
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

  typedef boost::shared_ptr<ScrollGrid3> Ptr;
  typedef boost::shared_ptr<const ScrollGrid3> ConstPtr;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ScrollGrid3() :
      radius_ijk_(0,0,0),
      box_(),
      origin_(0, 0, 0),
      min_world_corner_ijk_(0, 0, 0),
      dimension_(0, 0, 0),
      num_cells_(0),
      strides_(0, 0, 0),
      scroll_offset_(0, 0, 0),
      last_ijk_(0, 0, 0),
      wrap_ijk_min_(0, 0, 0),
      wrap_ijk_max_(0, 0, 0),
      unwrap_ijk_(0, 0, 0),
      resolution_(0)
  { }

  ScrollGrid3(const Vec3& center,
              const Vec3Ix& dimension,
              Scalar resolution,
              bool x_fastest=false) :
      radius_ijk_(dimension/2),
      box_(center-(radius_ijk_.cast<Scalar>()*resolution),
           center+(radius_ijk_.cast<Scalar>()*resolution)),
      origin_(center-box_.radius()),
      dimension_(dimension),
      num_cells_(dimension.prod()),
      scroll_offset_(0, 0, 0),
      last_ijk_(scroll_offset_ + dimension_),
      wrap_ijk_min_(0, 0, 0),
      wrap_ijk_max_(0, 0, 0),
      unwrap_ijk_(0, 0, 0),
      resolution_(resolution)
  {
    // calculate coordinates of min corner in ijk
    Scalar m = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    Vec3 m3(m, m, m);
    min_world_corner_ijk_ = this->world_to_grid(m3);

    if (x_fastest) {
      strides_ = Vec3Ix(1, dimension[0], dimension.head<2>().prod());
    } else {
      strides_ = Vec3Ix(dimension.tail<2>().prod(), dimension[2], 1);
    }

    this->update_wrap_ijk();
  }

  virtual ~ScrollGrid3() { }

  ScrollGrid3(const ScrollGrid3& other) :
      box_(other.box_),
      origin_(other.origin_),
      min_world_corner_ijk_(other.min_world_corner_ijk_),
      dimension_(other.dimension_),
      num_cells_(other.num_cells_),
      strides_(other.strides_),
      scroll_offset_(other.scroll_offset_),
      last_ijk_(other.last_ijk_),
      wrap_ijk_min_(other.wrap_ijk_min_),
      wrap_ijk_max_(other.wrap_ijk_max_),
      unwrap_ijk_(other.unwrap_ijk_),
      resolution_(other.resolution_)
  {
  }

  ScrollGrid3& operator=(const ScrollGrid3& other) {
    if (this==&other) { return *this; }
    box_ = other.box_;
    origin_ = other.origin_;
    min_world_corner_ijk_ = other.min_world_corner_ijk_;
    dimension_ = other.dimension_;
    num_cells_ = other.num_cells_;
    strides_ = other.strides_;
    scroll_offset_ = other.scroll_offset_;
    last_ijk_ = other.last_ijk_;
    wrap_ijk_min_ = other.wrap_ijk_min_;
    wrap_ijk_max_ = other.wrap_ijk_max_;
    unwrap_ijk_ = other.unwrap_ijk_;
    resolution_ = other.resolution_;
    return *this;
  }

public:

  void reset(const Vec3& center,
             const Vec3Ix& dimension,
             Scalar resolution,
             bool x_fastest=false) {

    radius_ijk_ = dimension/2;
    box_.set_center(center);
    box_.set_radius(radius_ijk_.cast<Scalar>() * resolution);
    origin_ = center - box_.radius();

    dimension_ = dimension;
    num_cells_ = dimension.prod();
    if (x_fastest) {
      strides_ = Vec3Ix(1, dimension[0], dimension.head<2>().prod());
    } else {
      strides_ = Vec3Ix(dimension.tail<2>().prod(), dimension[2], 1);
    }
    scroll_offset_.setZero();
    last_ijk_ = scroll_offset_ + dimension_;

    this->update_wrap_ijk();

    resolution_ = resolution;

    Vec3 m;
    m[0] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    m[1] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    m[2] = -static_cast<Scalar>(std::numeric_limits<uint16_t>::max()/2)*resolution_;
    min_world_corner_ijk_ = this->world_to_grid(m);

  }

  /**
   * Is inside 3D box containing grid?
   * @param pt point in same frame as center (probably world_view)
   */
  bool is_inside_box(const Vec3& pt) const {
    return box_.contains(pt);
  }

  template<class PointT>
  bool is_inside_box(const PointT& pt) const {
    return box_.contains(ca::point_cast<Vec3>(pt));
  }

  /**
   * is i, j, k inside the grid limits?
   */
  bool is_inside_grid(const Vec3Ix& grid_ix) const {
    return ((grid_ix.array() >= scroll_offset_.array()).all() &&
            (grid_ix.array() < (scroll_offset_+dimension_).array()).all());
  }

  bool is_inside_grid(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    return this->is_inside_grid(Vec3Ix(i, j, k));
  }

  /**
   * scroll grid.
   * updates bounding box and offset_cells.
   * @param offset_cells. how much to scroll. offset_cells is a signed integral.
   *
   */
  void just_scroll(const Vec3Ix& offset_cells) {
    Vec3Ix new_offset = scroll_offset_ + offset_cells;
    box_.translate((offset_cells.cast<Scalar>()*resolution_));
    scroll_offset_ = new_offset;
    last_ijk_ = scroll_offset_ + dimension_;

    this->update_wrap_ijk();
  }

/**
   * functionally same as just_scroll.
   * special case of scroll_and_clear_and_fix.
   */
  void scroll(const Vec3Ix& offset_cells) {
    ClearCellsFun nullclear;
    FixEdgesFun nullfix;
    this->scroll_and_clear_and_fix(offset_cells, nullclear, nullfix);
  }

/**
   * scroll grid by offset_cells and call clear_cells_fun on outgoing/incoming
   * cells.
   * special case of scroll_and_clear_and_fix.
   */  
  
  void scroll_and_clear(const Vec3Ix& offset_cells,
                        const ClearCellsFun& clear_cells_fun) {
    FixEdgesFun nullfix;
    this->scroll_and_clear_and_fix(offset_cells, clear_cells_fun, nullfix);
  }

/**
   * scroll grid by offset_cells, call fix_edges_fun on outgoing/incoming
   * edges, and clear_cells_fun on outgoing/incoming cells.
   */  
  void scroll_and_clear_and_fix(const Vec3Ix& offset_cells,
                                const ClearCellsFun& clear_cells_fun,
                                const FixEdgesFun& fix_edges_fun) {

    Vec3Ix new_offset = scroll_offset_ + offset_cells;

    // check if there is overlap between current box and box after scroll.
    // if there is not, then the whole box must be wiped out.
    if (( abs(offset_cells[0]) >= dimension_[0] ) ||
        ( abs(offset_cells[1]) >= dimension_[1] ) ||
        ( abs(offset_cells[2]) >= dimension_[2] ) ) {

      clear_cells_fun(scroll_offset_, scroll_offset_+dimension_);

      // not sure if *all* edges must be fixed or none of them should.
      // according to current logic none of them should.
      // TODO caveat: there is an -1 offset in all the edges calculations.
      // also note: earlier logic completely ignored this case.

    } else {
      if (offset_cells[0] > 0) {
        Vec3Ix finish(new_offset[0],
                      scroll_offset_[1] + dimension_[1],
                      scroll_offset_[2] + dimension_[2]);
        clear_cells_fun(scroll_offset_, finish);
        fix_edges_fun(0, finish[0], scroll_offset_[0]+dimension_[0]-1);
      } else if (offset_cells[0] < 0) {
        Vec3Ix start(scroll_offset_[0]+dimension_[0]+offset_cells[0],
                     scroll_offset_[1],
                     scroll_offset_[2]);
        Vec3Ix finish = scroll_offset_ + dimension_;
        clear_cells_fun(start, finish);
        fix_edges_fun(0, start[0]-1, scroll_offset_[0]);
      }

      if (offset_cells[1] > 0) {
        Vec3Ix finish(scroll_offset_[0] + dimension_[0],
                      new_offset[1],
                      scroll_offset_[2] + dimension_[2]);
        clear_cells_fun(scroll_offset_, finish);
        fix_edges_fun(1, finish[1], scroll_offset_[1]+dimension_[1]-1);
      } else if(offset_cells[1] < 0) {
        Vec3Ix start(scroll_offset_[0],
                     scroll_offset_[1]+dimension_[1]+offset_cells[1],
                     scroll_offset_[2]);
        Vec3Ix finish = scroll_offset_ + dimension_;
        clear_cells_fun(start, finish);
        fix_edges_fun(1, start[1]-1, scroll_offset_[1]);
      }

      if (offset_cells[2] > 0) {
        Vec3Ix finish(scroll_offset_[0] + dimension_[0],
                      scroll_offset_[1] + dimension_[1],
                      new_offset[2]);
        clear_cells_fun(scroll_offset_, finish);
        fix_edges_fun(2, finish[2], scroll_offset_[2]+dimension_[2]-1);
      } else if(offset_cells[2] < 0) {
        Vec3Ix start(scroll_offset_[0],
                     scroll_offset_[1],
                     scroll_offset_[2]+dimension_[2]+offset_cells[2]);
        Vec3Ix finish = scroll_offset_ + dimension_;
        clear_cells_fun(start, finish);
        fix_edges_fun(2, start[2]-1, scroll_offset_[2]);
      }
    }

    box_.translate((offset_cells.cast<Scalar>()*resolution_));
    scroll_offset_ = new_offset;
    last_ijk_ = scroll_offset_ + dimension_;
    this->update_wrap_ijk();

  }

  /**
   * get boxes to clear if scrolling by offset_cells.
   * Call this *before* scroll().
   * @param clear_i_min min corner of obsolete region in grid
   * @param clear_i_max max corner of obsolete region in grid
   * same for j and k
   * Note that boxes may overlap.
   *
   * Note: this may be deprecated in favor of the clearfun callbacks
   * in scroll.
   */
  void get_clear_boxes(const Vec3Ix& offset_cells,
                       Vec3Ix& clear_i_min, Vec3Ix& clear_i_max,
                       Vec3Ix& clear_j_min, Vec3Ix& clear_j_max,
                       Vec3Ix& clear_k_min, Vec3Ix& clear_k_max) {

    clear_i_min.setZero();
    clear_j_min.setZero();
    clear_k_min.setZero();
    clear_i_max.setZero();
    clear_j_max.setZero();
    clear_k_max.setZero();

    // first check if there is overlap between current and box after scroll.
    // if there is not, then the whole box must be wiped out.
    if (( abs(offset_cells[0]) >= dimension_[0] ) ||
        ( abs(offset_cells[1]) >= dimension_[1] ) ||
        ( abs(offset_cells[2]) >= dimension_[2] ) ) {
      clear_i_min = scroll_offset_;
      clear_i_max = scroll_offset_ + dimension_;
      return;
    }

    Vec3Ix new_offset = scroll_offset_ + offset_cells;

    // X axis
    if (offset_cells[0] > 0) {
      clear_i_min = scroll_offset_;
      clear_i_max = Vec3Ix(new_offset[0],
                           scroll_offset_[1]+dimension_[1],
                           scroll_offset_[2]+dimension_[2]);
    } else if (offset_cells[0] < 0) {
      clear_i_min = Vec3Ix(scroll_offset_[0]+dimension_[0]+offset_cells[0],
                           scroll_offset_[1],
                           scroll_offset_[2]);
      clear_i_max = scroll_offset_ + dimension_;
    }

    // Y axis
    if (offset_cells[1] > 0) {
      clear_j_min = scroll_offset_;
      clear_j_max = Vec3Ix(scroll_offset_[0]+dimension_[0],
                           new_offset[1],
                           scroll_offset_[2]+dimension_[2]);
    } else if (offset_cells[1] < 0) {
      clear_j_min = Vec3Ix(scroll_offset_[0],
                           scroll_offset_[1]+dimension_[1]+offset_cells[1],
                           scroll_offset_[2]);
      clear_j_max = scroll_offset_ + dimension_;
    }

    // Z axis
    if (offset_cells[2] > 0) {
      clear_k_min = scroll_offset_;
      clear_k_max = Vec3Ix(scroll_offset_[0]+dimension_[0],
                           scroll_offset_[1]+dimension_[1],
                           new_offset[2]);
    } else if(offset_cells[2] < 0) {
      clear_k_min = Vec3Ix(scroll_offset_[0],
                           scroll_offset_[1],
                           scroll_offset_[2]+dimension_[2]+offset_cells[2]);
      clear_k_max = scroll_offset_ + dimension_;
    }

  }

  /**
   * Given position in world coordinates, return grid coordinates.
   * (grid coordinates are not wrapped to be inside grid!)
   * Note: does not check if point is inside grid.
   */
  Vec3Ix world_to_grid(const Vec3& xyz) const {
    Vec3 tmp = ((xyz - origin_).array() - 0.5*resolution_)/resolution_;
    return Vec3Ix(round(tmp.x()), round(tmp.y()), round(tmp.z()));
  }

  Vec3Ix world_to_grid(Scalar x, Scalar y, Scalar z) const {
    return this->world_to_grid(Vec3(x, y, z));
  }

  /**
   * Like world to grid but xyz are offset by scroll grid center.
   * DON'T USE OR YOU WILL SCREW UP.
   */
  Vec3Ix offset_world_to_grid(const Vec3& xyz) const {
    Vec3 tmp = ((xyz + box_.center() - origin_).array() - 0.5*resolution_)/resolution_;
    return Vec3Ix(round(tmp.x()), round(tmp.y()), round(tmp.z()));
  }

  Vec3Ix offset_world_to_grid(Scalar x, Scalar y, Scalar z) const {
    return this->offset_world_to_grid(Vec3(x, y, z));
  }

  Vec3 grid_to_world(const Vec3Ix& grid_ix) const {
    Vec3 w((grid_ix.cast<Scalar>()*resolution_ + origin_).array() + 0.5*resolution_);
    return w;
  }

  Vec3 grid_to_world(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    return this->grid_to_world(Vec3Ix(i, j, k));
  }

  /**
   * Translate grid indices to an address in linear memory.
   * Does not check if grid_ix is inside current grid box.
   * Assumes C-order, x the slowest and z the fastest.
   */
  mem_ix_t grid_to_mem_slow(const Vec3Ix& grid_ix) const {
    Vec3Ix grid_ix2(ca::mod_wrap(grid_ix[0], dimension_[0]),
                    ca::mod_wrap(grid_ix[1], dimension_[1]),
                    ca::mod_wrap(grid_ix[2], dimension_[2]));
    return strides_.dot(grid_ix2);
  }

  /**
   * Faster than grid_to_mem_slow, as it avoids modulo.
   * But it only works if the grid_ix are inside the bounding box.
   * Hopefully branch prediction kicks in when using this in a loop.
   */
  mem_ix_t grid_to_mem(const Vec3Ix& grid_ix) const {
    ROS_ASSERT( this->is_inside_grid(grid_ix) );

    Vec3Ix grid_ix2(grid_ix);

    if (grid_ix2[0] >= wrap_ijk_max_[0]) { grid_ix2[0] -= wrap_ijk_max_[0]; } else { grid_ix2[0] -= wrap_ijk_min_[0]; }
    if (grid_ix2[1] >= wrap_ijk_max_[1]) { grid_ix2[1] -= wrap_ijk_max_[1]; } else { grid_ix2[1] -= wrap_ijk_min_[1]; }
    if (grid_ix2[2] >= wrap_ijk_max_[2]) { grid_ix2[2] -= wrap_ijk_max_[2]; } else { grid_ix2[2] -= wrap_ijk_min_[2]; }

    mem_ix_t mem_ix2 = strides_.dot(grid_ix2);

    return mem_ix2;
  }

  mem_ix_t grid_to_mem_slow(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    return this->grid_to_mem_slow(Vec3Ix(i, j, k));
  }

  mem_ix_t grid_to_mem(grid_ix_t i, grid_ix_t j, grid_ix_t k) const {
    return grid_to_mem( Vec3Ix(i, j, k) );
  }

  /**
   * TODO should we do this in the sparse_array3 module?
   */
  uint64_t grid_to_hash(const Vec3Ix& grid_ix) const {
    // grid2 should be all positive
    Vec3Ix grid2(grid_ix - min_world_corner_ijk_);

    uint64_t hi = static_cast<uint64_t>(grid2[0]);
    uint64_t hj = static_cast<uint64_t>(grid2[1]);
    uint64_t hk = static_cast<uint64_t>(grid2[2]);
    uint64_t h = (hi << 48) | (hj << 32) | (hk << 16);
    return h;
  }

  Vec3Ix hash_to_grid(uint64_t hix) const {
    uint64_t hi = (hix & 0xffff000000000000) >> 48;
    uint64_t hj = (hix & 0x0000ffff00000000) >> 32;
    uint64_t hk = (hix & 0x00000000ffff0000) >> 16;
    Vec3Ix grid_ix(hi, hj, hk);
    grid_ix += min_world_corner_ijk_;
    return grid_ix;
  }

  /**
   * Note that no bound check is performed!
   */
  mem_ix_t world_to_mem(const Vec3& xyz) const {
    Vec3Ix gix(this->world_to_grid(xyz));
    return this->grid_to_mem(gix);
  }

  Vec3Ix mem_to_grid(grid_ix_t mem_ix) const {
    // TODO does this work for x-fastest strides?
    grid_ix_t i = mem_ix/strides_[0];
    mem_ix -= i*strides_[0];
    grid_ix_t j = mem_ix/strides_[1];
    mem_ix -= j*strides_[1];
    grid_ix_t k = mem_ix;

    if (i < unwrap_ijk_[0]) { i += wrap_ijk_max_[0]; } else { i += wrap_ijk_min_[0]; }
    if (j < unwrap_ijk_[1]) { j += wrap_ijk_max_[1]; } else { j += wrap_ijk_min_[1]; }
    if (k < unwrap_ijk_[2]) { k += wrap_ijk_max_[2]; } else { k += wrap_ijk_min_[2]; }

    return (Vec3Ix(i, j, k));

  }

 public:
  grid_ix_t dim_i() const { return dimension_[0]; }
  grid_ix_t dim_j() const { return dimension_[1]; }
  grid_ix_t dim_k() const { return dimension_[2]; }
  grid_ix_t first_i() const { return scroll_offset_[0]; }
  grid_ix_t first_j() const { return scroll_offset_[1]; }
  grid_ix_t first_k() const { return scroll_offset_[2]; }
  grid_ix_t last_i() const { return last_ijk_[0]; }
  grid_ix_t last_j() const { return last_ijk_[1]; }
  grid_ix_t last_k() const { return last_ijk_[2]; }
  const Vec3Ix& scroll_offset() const { return scroll_offset_; }
  const Vec3Ix& unwrap_ijk() const { return unwrap_ijk_; }
//   const Vec3Ix& unwrap_ijk_() const { return unwrap_ijk_; }
  const Vec3Ix& radius_ijk() const { return radius_ijk_; }
  const Vec3Ix& dimension() const { return dimension_; }
  const Vec3& radius() const { return box_.radius(); }
  const Vec3& origin() const { return origin_; }
  Vec3 min_pt() const { return box_.min_pt(); }
  Vec3 max_pt() const { return box_.max_pt(); }
  const Vec3& center() const { return box_.center(); }
  Scalar resolution() const { return resolution_; }
  const ca::scrollgrid::Box<Scalar, 3>& box() const { return box_; }

  grid_ix_t num_cells() const { return num_cells_; }

 private:

  /**
   * (Re)calculate indices where wrapping occurs for faster lookups.
   * This basically avoids modulo operations.
   * Must be called each time scrollgrid moves.
   */
  void update_wrap_ijk() {
    wrap_ijk_min_[0] = floor(static_cast<float>(scroll_offset_[0])/dimension_[0])*dimension_[0];
    wrap_ijk_min_[1] = floor(static_cast<float>(scroll_offset_[1])/dimension_[1])*dimension_[1];
    wrap_ijk_min_[2] = floor(static_cast<float>(scroll_offset_[2])/dimension_[2])*dimension_[2];

    wrap_ijk_max_[0] = floor(static_cast<float>(scroll_offset_[0]+dimension_[0])/dimension_[0])*dimension_[0];
    wrap_ijk_max_[1] = floor(static_cast<float>(scroll_offset_[1]+dimension_[1])/dimension_[1])*dimension_[1];
    wrap_ijk_max_[2] = floor(static_cast<float>(scroll_offset_[2]+dimension_[2])/dimension_[2])*dimension_[2];

    unwrap_ijk_[0] = ca::mod_wrap(scroll_offset_[0], dimension_[0]);
    unwrap_ijk_[1] = ca::mod_wrap(scroll_offset_[1], dimension_[1]);
    unwrap_ijk_[2] = ca::mod_wrap(scroll_offset_[2], dimension_[2]);

  }

 private:

  // discrete radius, used when calculating box dimensions
  Vec3Ix radius_ijk_;

  // 3d box enclosing grid. In whatever coordinates were given (probably
  // world_view)
  ca::scrollgrid::Box<Scalar, 3> box_;

  // xyz position of grid origin in world_xyz frame.
  // does not move when scrolling.
  // initialized as (center - box.radius).
  Vec3 origin_;

  // minimum world corner in ijk. used for hash
  Vec3Ix min_world_corner_ijk_;

  // number of grid cells along each axis
  Vec3Ix dimension_;

  // number of cells
  grid_ix_t num_cells_;

  // grid strides to translate from linear to 3D layout.
  // C-ordering, ie x slowest, z fastest.
  Vec3Ix strides_;

  // to keep track of scrolling along z.
  Vec3Ix scroll_offset_;

  // redundant but actually seems to have a performance benefit
  // should always be dimension + offset
  Vec3Ix last_ijk_;

  // for grid_to_mem. the points where the grid crosses modulo boundaries. // nearly boundary pt
  Vec3Ix wrap_ijk_min_;
  Vec3Ix wrap_ijk_max_;

  // delimits when extra offset needs to be added when unwrapping  offset % dimension 余数
  Vec3Ix unwrap_ijk_;


  // size of grid cells
  Scalar resolution_;

};

typedef ScrollGrid3<float> ScrollGrid3f;
typedef ScrollGrid3<double> ScrollGrid3d;

} /* ca */

#endif /* end of include guard: SCROLLGRID3_HPP_I9SAOOSJ */
