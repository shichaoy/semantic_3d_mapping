/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#ifndef ROT_CAST_HPP_TH3FOBZQ
#define ROT_CAST_HPP_TH3FOBZQ

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/integral_constant.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>

/**
 * TODO
 * include other rotation forms.
 * btMatrix3x3 (getRPY, getEulerYPR)
 * btQuaternion (setRPY and setEulerZYX)
 * note btquaternion takes matrix 3x3
 * KDL::Rotation?
 * also see: transformations.py or RVC
 * also see: tf_conversions
 */

namespace ca
{

namespace detail
{

///////////////////////////////////////////////////////////////////////////////
// traits

template<typename T>
struct rot_quat_member_getset : public boost::false_type { };

template<typename T>
struct rot_quat_fun_member_getset : public boost::false_type { };

template<typename T>
struct rot_quat_fun_member_get : public boost::false_type { };

template<typename T>
struct rot_quat_xyzw_array_getset : public boost::false_type { };

template<typename T>
struct rot_quat_xyzw_ctor_set : public boost::false_type { };

///////////////////////////////////////////////////////////////////////////////
// implementations

// geometry_msgs
template<>
struct rot_quat_member_getset<geometry_msgs::Quaternion> : public boost::true_type { };

// eigen
template<class Scalar>
struct rot_quat_fun_member_getset<Eigen::Quaternion<Scalar> > : public boost::true_type { };

// tf/bt
template<>
struct rot_quat_xyzw_ctor_set<tf::Quaternion> : public boost::true_type { };

template<>
struct rot_quat_fun_member_get<tf::Quaternion> : public boost::true_type { };

} // detail

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x = src.x;
  tgt.y = src.y;
  tgt.z = src.z;
  tgt.w = src.w;
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_xyzw_array_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt[0] = src.x;
  tgt[1] = src.y;
  tgt[2] = src.z;
  tgt[3] = src.w;
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_xyzw_array_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x = src[0];
  tgt.y = src[1];
  tgt.z = src[2];
  tgt.w = src[3];
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_fun_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x() = src.x;
  tgt.y() = src.y;
  tgt.z() = src.z;
  tgt.w() = src.w;
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_fun_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x = src.x();
  tgt.y = src.y();
  tgt.z = src.z();
  tgt.w = src.w();
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_fun_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_xyzw_ctor_set<Target> >::type* dummy2 = 0
               ) {
  return Target(src.x(), src.y(), src.z(), src.w());
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_member_getset<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_xyzw_ctor_set<Target> >::type* dummy2 = 0
               ) {
  return Target(src.x, src.y, src.z, src.w);
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_fun_member_get<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_xyzw_ctor_set<Target> >::type* dummy2 = 0
               ) {
  return Target(src.x(), src.y(), src.z(), src.w());
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_fun_member_get<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x = src.x();
  tgt.y = src.y();
  tgt.z = src.z();
  tgt.w = src.w();
  return tgt;
}

template<typename Target, typename Source>
Target rot_cast(const Source& src,
                typename boost::enable_if< detail::rot_quat_fun_member_get<Source> >::type* dummy1 = 0,
                typename boost::enable_if< detail::rot_quat_fun_member_getset<Target> >::type* dummy2 = 0
               ) {
  Target tgt;
  tgt.x() = src.x();
  tgt.y() = src.y();
  tgt.z() = src.z();
  tgt.w() = src.w();
  return tgt;
}

} /* ca */

#endif /* end of include guard: ROT_CAST_HPP_TH3FOBZQ */
