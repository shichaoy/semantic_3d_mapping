/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/

#ifndef NEA_TO_PCL_HPP_TFK4ZHX8
#define NEA_TO_PCL_HPP_TFK4ZHX8

#include <boost/foreach.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geom_cast/geom_cast.hpp>
#include <geom_cast_extra/geom_cast_extra.h>

namespace ca
{

/**
 * Extract xyz fields of nea pointcloud2.
 * Assumes there are adjacent x, y and z fields somewhere in the cloud of type Scalar.
 * Other fields are discarded.
 */
template<class Scalar>
void NeaPc2ToPclXyz(const sensor_msgs::PointCloud2& cloud,
                    pcl::PointCloud<pcl::PointXYZ>& xyz) {
  typedef Eigen::Matrix<Scalar, 3, 1> vec3;

  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );
  if (cloud.width==0) { xyz.clear(); return; }
  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  static uint32_t x_offset = cloud.fields[x_field_ix].offset;
  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr != end_ptr;
       ptr += cloud.point_step) {
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_offset));
    pcl::PointXYZ p = ca::point_cast<pcl::PointXYZ>(ep);
    xyz.push_back(p);
  }
}

/**
 * Extract xyz fields of nea pointcloud2, dynamically choose float/double.
 * Extracts adjacent xyz fields from anywhere.
 * Dynamically chooses input type based on type of "x" field.
 */
inline
void NeaPc2ToPclXyz(const sensor_msgs::PointCloud2& cloud,
                    pcl::PointCloud<pcl::PointXYZ>& xyz) {
  if (cloud.width==0) { xyz.clear(); return; }

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT32) {
    NeaPc2ToPclXyz<float>(cloud, xyz);
  } else if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT64) {
    NeaPc2ToPclXyz<double>(cloud, xyz);
  } else {
    ROS_ERROR("Unknown type");
  }
}

/**
 * Extract xyz fields of nea pointcloud2, selecting indices.
 * Extract Scalar xyz from a subset of points in the cloud.
 */
template<class Scalar>
void NeaPc2ToPclXyz(const sensor_msgs::PointCloud2& cloud,
                    const std::vector<int>& indices,
                    pcl::PointCloud<pcl::PointXYZ>& xyz) {
  typedef Eigen::Matrix<Scalar, 3, 1> vec3;

  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );
  xyz.clear();
  if (cloud.width == 0) { return; }
  if (indices.empty()) { return; }
  xyz.reserve(indices.size());

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  static uint32_t x_offset = cloud.fields[x_field_ix].offset;

  BOOST_FOREACH(int ix, indices) {
    const uint8_t *ptr = &(cloud.data[0]) + ix*cloud.point_step;
    ROS_ASSERT(ptr < &(cloud.data[0]) + cloud.row_step);
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_offset));
    pcl::PointXYZ p = ca::point_cast<pcl::PointXYZ>(ep);
    xyz.push_back(p);
  }

}

/**
 * Extract nea fields of indinces, dynamically select double/float and indices.
 * Dynamic input type dispatch.
 */
inline
void NeaPc2ToPclXyz(const sensor_msgs::PointCloud2& cloud,
                    const std::vector<int>& indices,
                    pcl::PointCloud<pcl::PointXYZ>& xyz) {
  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT32) {
    NeaPc2ToPclXyz<float>(cloud, indices, xyz);
  } else if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT64) {
    NeaPc2ToPclXyz<double>(cloud, indices, xyz);
  } else {
    ROS_ERROR("Unknown type");
  }
}

/**
 * Extract xyz and reflectance.
 * Extracts x,y,z and reflectance fields into a PointXYZI cloud.
 */
template<class Scalar>
void NeaPc2ToPclXyzi(const sensor_msgs::PointCloud2& cloud,
                     pcl::PointCloud<pcl::PointXYZI>& xyzi) {
  typedef Eigen::Matrix<Scalar, 3, 1> vec3;
  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );

  xyzi.clear();
  if (cloud.width == 0) { return; }
  xyzi.reserve(cloud.width);

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  static int reflectance_field_ix = pcl::getFieldIndex(cloud, "reflectance");
  ROS_ASSERT(reflectance_field_ix >= 0);
  static uint32_t x_offset = cloud.fields[x_field_ix].offset;
  static uint32_t reflectance_offset = cloud.fields[reflectance_field_ix].offset;

  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr != end_ptr;
       ptr += cloud.point_step) {
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_offset));
    float reflectance(*reinterpret_cast<const float*>(ptr+reflectance_offset));
    pcl::PointXYZI p;
    p.x = ep.x();
    p.y = ep.y();
    p.z = ep.z();
    p.intensity = reflectance;
    xyzi.push_back(p);
  }
}

/**
 * Extract xyz and reflectance, dynamically choose float/double.
 */
inline
void NeaPc2ToPclXyzi(const sensor_msgs::PointCloud2& cloud,
                     pcl::PointCloud<pcl::PointXYZI>& xyzi) {

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  if (cloud.fields[x_field_ix].datatype==sensor_msgs::PointField::FLOAT32) {
    NeaPc2ToPclXyzi<float>(cloud, xyzi);
  } else if (cloud.fields[x_field_ix].datatype==sensor_msgs::PointField::FLOAT64) {
    NeaPc2ToPclXyzi<double>(cloud, xyzi);
  } else {
    ROS_ERROR("Unknown type");
  }
}

/**
 * Extract xyz_origin.
 * Extracts the x_origin, y_origin and z_origin points.
 */
template<class Scalar>
void NeaPc2ViewpointToPclXyz(const sensor_msgs::PointCloud2& cloud,
                             pcl::PointCloud<pcl::PointXYZ>& xyz) {

  typedef Eigen::Matrix<Scalar, 3, 1> vec3;

  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );
  static int x_orig_field_ix = pcl::getFieldIndex(cloud, "x_origin");
  ROS_ASSERT(x_orig_field_ix >= 0);
  static uint32_t x_origin_offset = cloud.fields[x_orig_field_ix].offset;

  xyz.clear();
  if (cloud.width == 0) { return; }
  xyz.reserve(cloud.width);

  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr != end_ptr;
       ptr += cloud.point_step) {
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_origin_offset));
    pcl::PointXYZ p = ca::point_cast<pcl::PointXYZ>(ep);
    xyz.push_back(p);
  }
}

/**
 * Extract xyz_origin, dynamically choose float/double.
 * Extracts the x_origin, y_origin and z_origin points.
 */
inline
void NeaPc2ViewpointToPclXyz(const sensor_msgs::PointCloud2& cloud,
                             pcl::PointCloud<pcl::PointXYZ>& xyz) {

  static int x_orig_field_ix = pcl::getFieldIndex(cloud, "x_origin");
  ROS_ASSERT(x_orig_field_ix >= 0);
  if (cloud.fields[x_orig_field_ix].datatype == sensor_msgs::PointField::FLOAT32) {
    NeaPc2ViewpointToPclXyz<float>(cloud, xyz);
  } else if (cloud.fields[x_orig_field_ix].datatype == sensor_msgs::PointField::FLOAT64) {
    NeaPc2ViewpointToPclXyz<double>(cloud, xyz);
  } else {
    ROS_ERROR("Unknown type");
  }

}

/**
 * Extract xyz_origin fields.
 * Extracts the x_origin, y_origin and z_origin points.
 */
template<class Scalar>
void NeaPc2ViewpointToPclXyz(const sensor_msgs::PointCloud2& cloud,
                             const std::vector<int>& indices,
                             pcl::PointCloud<pcl::PointXYZ>& xyz) {
  typedef Eigen::Matrix<Scalar, 3, 1> vec3;

  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );
  static int x_orig_field_ix = pcl::getFieldIndex(cloud, "x_origin");
  ROS_ASSERT(x_orig_field_ix >= 0);
  static uint32_t x_origin_offset = cloud.fields[x_orig_field_ix].offset;

  xyz.clear();
  if (cloud.width == 0) { return; }
  if (indices.empty()) { return; }
  xyz.reserve(indices.size());

  BOOST_FOREACH(int ix, indices) {
    const uint8_t *ptr = &(cloud.data[0]) + ix*cloud.point_step;
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_origin_offset));
    pcl::PointXYZ p = ca::point_cast<pcl::PointXYZ>(ep);
    xyz.push_back(p);
  }
}

inline
void NeaPc2ViewpointToPclXyz(const sensor_msgs::PointCloud2& cloud,
                             const std::vector<int>& indices,
                             pcl::PointCloud<pcl::PointXYZ>& xyz) {
  static int x_orig_field_ix = pcl::getFieldIndex(cloud, "x_origin");
  ROS_ASSERT(x_orig_field_ix >= 0);
  if (cloud.fields[x_orig_field_ix].datatype==sensor_msgs::PointField::FLOAT32) {
    NeaPc2ViewpointToPclXyz<float>(cloud, indices, xyz);
  } else if (cloud.fields[x_orig_field_ix].datatype==sensor_msgs::PointField::FLOAT64) {
    NeaPc2ViewpointToPclXyz<double>(cloud, indices, xyz);
  } else {
    ROS_ERROR("Unknown type");
  }
}

/**
 * Extract xyz and xyz_origin to PointWithViewpoint (x, y, z, vp_x, vp_y, vp_z).
 * Extracts x,y,z and x_origin, y_origin, z_origin to PointWithViewpoint cloud.
 */
template<class Scalar>
void NeaPc2ToPclXyzvp(const sensor_msgs::PointCloud2& cloud,
                      pcl::PointCloud<pcl::PointWithViewpoint>& xyzvp) {

  typedef Eigen::Matrix<Scalar, 3, 1> vec3;

  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );
  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);

  static int x_orig_field_ix = pcl::getFieldIndex(cloud, "x_origin");
  ROS_ASSERT(x_orig_field_ix >= 0);

  static uint32_t x_offset = cloud.fields[x_field_ix].offset;
  static uint32_t x_orig_offset = cloud.fields[x_orig_field_ix].offset;

  xyzvp.clear();

  if (cloud.width == 0) { return; }
  xyzvp.reserve(cloud.width);

  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr != end_ptr;
       ptr += cloud.point_step) {
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_offset));
    Eigen::Map<const vec3> evp(reinterpret_cast<const Scalar*>(ptr+x_orig_offset));
    pcl::PointWithViewpoint pvp;
    pvp.x = ep.x();
    pvp.y = ep.y();
    pvp.z = ep.z();
    pvp.vp_x = evp.x();
    pvp.vp_y = evp.y();
    pvp.vp_z = evp.z();
    xyzvp.push_back(pvp);
  }
}

/**
 * Extract xyz and xyz_origin to PointWithViewpoint (x, y, z, vp_x, vp_y, vp_z).
 */
inline
void NeaPc2ToPclXyzvp(const sensor_msgs::PointCloud2& cloud,
                      pcl::PointCloud<pcl::PointWithViewpoint>& xyzvp) {
  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT32) {
    NeaPc2ToPclXyzvp<float>(cloud, xyzvp);
  } else if (cloud.fields[x_field_ix].datatype == sensor_msgs::PointField::FLOAT64) {
    NeaPc2ToPclXyzvp<double>(cloud, xyzvp);
  } else {
    ROS_ERROR("Unknown type for xyz");
  }
}

/**
 * Extracts x,y,z and label fields into a PointXYZL cloud.
 */
template<class Scalar>
void NeaPc2ToPclXyzl(const sensor_msgs::PointCloud2& cloud,
                     pcl::PointCloud<pcl::PointXYZL>& xyzl) {
  typedef Eigen::Matrix<Scalar, 3, 1> vec3;
  ROS_ASSERT( pcl::traits::asEnum<Scalar>::value == cloud.fields[0].datatype );

  xyzl.clear();
  if (cloud.width == 0) { return; }
  xyzl.reserve(cloud.width);

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  static int label_field_ix = pcl::getFieldIndex(cloud, "label");
  ROS_ASSERT(label_field_ix >= 0);

  static uint32_t x_offset = cloud.fields[x_field_ix].offset;
  static uint32_t label_offset = cloud.fields[label_field_ix].offset;

  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr != end_ptr;
       ptr += cloud.point_step) {
    Eigen::Map<const vec3> ep(reinterpret_cast<const Scalar*>(ptr+x_offset));
    uint8_t label(*reinterpret_cast<const uint8_t*>(ptr+label_offset));
    pcl::PointXYZL p;
    p.x = ep.x();
    p.y = ep.y();
    p.z = ep.z();
    p.label = static_cast<uint32_t>(label);
    xyzl.push_back(p);
  }
}

inline
void NeaPc2ToPclXyzl(const sensor_msgs::PointCloud2& cloud,
                     pcl::PointCloud<pcl::PointXYZL>& xyzl) {

  static int x_field_ix = pcl::getFieldIndex(cloud, "x");
  ROS_ASSERT(x_field_ix >= 0);
  if (cloud.fields[x_field_ix].datatype==sensor_msgs::PointField::FLOAT32) {
    NeaPc2ToPclXyzl<float>(cloud, xyzl);
  } else if (cloud.fields[x_field_ix].datatype==sensor_msgs::PointField::FLOAT64) {
    NeaPc2ToPclXyzl<double>(cloud, xyzl);
  } else {
    ROS_FATAL("Unknown type");
  }
}

} /* ca */

#endif /* end of include guard: NEA_TO_PCL_HPP_TFK4ZHX8 */
