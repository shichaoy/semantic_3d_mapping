/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */

#ifndef _GEOM_CAST_EXTRA_GEOM_CAST_EXTRA_H_
#define _GEOM_CAST_EXTRA_GEOM_CAST_EXTRA_H_

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/integral_constant.hpp>

#include <Eigen/Core>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>

#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#include <geom_cast/point_cast.hpp>

namespace ca {
namespace detail {

// pcl get
template<>
struct xyz_member_get<pcl::PointXYZ> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZI> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGB> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGBA> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointNormal> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZRGBNormal> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointXYZL> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithViewpoint> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithRange> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointWithScale> : public boost::true_type { };

template<>
struct xyz_member_get<pcl::PointSurfel> : public boost::true_type { };

// pcl set
template<>
struct xyz_member_set<pcl::PointXYZ> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZI> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGB> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGBA> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointNormal> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZRGBNormal> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointXYZL> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithViewpoint> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithRange> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointWithScale> : public boost::true_type { };

template<>
struct xyz_member_set<pcl::PointSurfel> : public boost::true_type { };

// opencv
template<class Scalar>
struct xyz_member_get<cv::Point3_<Scalar> > : public boost::true_type { };

template<class Scalar>
struct xyz_member_set<cv::Point3_<Scalar> > : public boost::true_type { };

template<class Scalar>
struct xyz_array_get< cv::Vec<Scalar, 3> > : public boost::true_type { };

template<class Scalar>
struct xyz_ctor_set< cv::Vec<Scalar, 3> > : public boost::true_type { };

///////////////////////////////////////////////////////////////////////////////
// 2D trait impl

// opencv
template<class Scalar>
struct xy_member_get<cv::Point_<Scalar> > : public boost::true_type { };

template<class Scalar>
struct xy_member_set<cv::Point_<Scalar> > : public boost::true_type { };

template<class Scalar>
struct xy_array_get<cv::Vec<Scalar, 2> > : public boost::true_type { };

template<class Scalar>
struct xy_ctor_set< cv::Vec<Scalar, 2> > : public boost::true_type { };

// pcl
template<>
struct xy_member_set<pcl::PointXY> : public boost::true_type { };

template<>
struct xy_member_get<pcl::PointXY> : public boost::true_type { };

}

} /* ca */

#endif
