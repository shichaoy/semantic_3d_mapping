#ifndef BOX_HPP_4WFLKG9Q
#define BOX_HPP_4WFLKG9Q

#include <assert.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace ca
{
namespace scrollgrid
{

/**
 * Describes an axis-aligned volume in space.
 */
template<typename Scalar, int Dim>
class Box {
 public:
  typedef Eigen::Matrix<Scalar, Dim, 1> Vec;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Box() {
    center_.setZero();
    radius_.setZero();
    bounds_[0].setZero();
    bounds_[1].setZero();
  }

  Box(const Vec& min_pt,
      const Vec& max_pt) :
      center_( (min_pt+max_pt)/2 ),
      radius_( (max_pt-min_pt)/2 )
  {
    bounds_[0] = min_pt;
    bounds_[1] = max_pt;
  }

  Box(const Box& other) :
      center_(other.center_),
      radius_(other.radius_)
  {
    bounds_[0] = other.bounds_[0];
    bounds_[1] = other.bounds_[1];
  }

  Box& operator=(const Box& other) {
    if (this==&other) { return *this; }
    center_ = other.center_;
    radius_ = other.radius_;
    bounds_[0] = other.bounds_[0];
    bounds_[1] = other.bounds_[1];
    return *this;
  }

  const Vec& min_pt() const { return bounds_[0]; }

  const Vec& max_pt() const { return bounds_[1]; }

  const Vec& bound(int ix) const {
    assert (ix >= 0);
    assert (ix < 2);
    return bounds_[ix];
  }

  void set_bound(int ix, const Vec& v) {
    assert (ix >= 0);
    assert (ix < 2);
    bounds_[ix] = v;
    center_ = (bounds_[0]+bounds_[1])/2;
    radius_ = (bounds_[0]-bounds_[1])/2;
  }

  void set_min_pt(const Vec& min_pt) {
    bounds_[0] = min_pt;
    center_ = (bounds_[0]+bounds_[1])/2;
    radius_ = (bounds_[0]-bounds_[1])/2;
  }

  void set_max_pt(const Vec& max_pt) {
    bounds_[1] = max_pt;
    center_ = (bounds_[0]+bounds_[1])/2;
    radius_ = (bounds_[0]-bounds_[1])/2;
  }

  const Vec& center() const {
    return center_;
  }

  const Vec& radius() const {
    return radius_;
  }

  void set_center(const Vec& v) {
    Vec delta = v - center_;
    this->translate(delta);
  }

  void set_radius(const Vec& v) {
    radius_ = v;
    bounds_[0] = center_ - radius_;
    bounds_[1] = center_ + radius_;
  }

  void translate(const Vec& v) {
    center_ += v;
    bounds_[0] += v;
    bounds_[1] += v;
  }

  bool contains(const Vec& v) const {
    return (bounds_[0].array() <= v.array()).all() &&
           (v.array() <= bounds_[1].array()).all();
  }

#if 0
  template<typename Scalar2>
  Box<Scalar2, Dim> cast() {
    Box<Scalar2, Dim> ret(bounds_[0].template cast<Scalar2>(),
                          bounds_[1].template cast<Scalar2>());
    return ret;
  }
#endif

 private:

  Vec center_;
  Vec radius_;
  Vec bounds_[2];
};

}
} /* ca */

#endif /* end of include guard: BOX_HPP_4WFLKG9Q */
