/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#ifndef TYPES_HPP_4KICFPUT
#define TYPES_HPP_4KICFPUT

#include <stdint.h>

//#include <boost/tuple/tuple.hpp>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <sensor_msgs/PointCloud2.h>

namespace ca
{

// these are mostly used here and there as abbreviations.
typedef pcl::PointXY P_XY;
typedef pcl::PointXYZ P_XYZ;
typedef pcl::PointXYZRGB P_XYZRGB;
typedef pcl::PointXYZI P_XYZI;
typedef pcl::PointXYZL P_XYZL;
typedef pcl::PointXYZINormal P_XYZIN;
typedef pcl::PointWithViewpoint P_XYZVP;

typedef pcl::PointCloud<pcl::PointXY> PC_XY;
typedef pcl::PointCloud<pcl::PointXYZ> PC_XYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PC_XYZRGB;
typedef pcl::PointCloud<pcl::PointXYZI> PC_XYZI;
typedef pcl::PointCloud<pcl::PointXYZL> PC_XYZL;
typedef pcl::PointCloud<pcl::PointXYZINormal> PC_XYZIN;
typedef pcl::PointCloud<pcl::PointWithViewpoint> PC_XYZVP;

typedef pcl::PCLPointCloud2 PPC2;
typedef sensor_msgs::PointCloud2 PC2;

} /* ca */

#endif /* end of include guard: TYPES_HPP_4KICFPUT */
