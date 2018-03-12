/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#ifndef NEA_PC_FORMAT_HPP_QTYJUJVB
#define NEA_PC_FORMAT_HPP_QTYJUJVB

#include <stdint.h>
#include <vector>

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_util/pcl_util.hpp"

namespace ca {
struct NeaPoint {
  double x;
  double y;
  double z;
  double x_origin;
  double y_origin;
  double z_origin;
  float range_variance;
  float x_variance;
  float y_variance;
  float z_variance;
  float reflectance;
  uint32_t time_sec;
  uint32_t time_nsec;
  uint8_t return_type;
  uint8_t label;
  uint8_t padding[2];

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct NeaPointf {
  float x;
  float y;
  float z;
  float x_origin;
  float y_origin;
  float z_origin;
  float range_variance;
  float x_variance;
  float y_variance;
  float z_variance;
  float reflectance;
  uint32_t time_sec;
  uint32_t time_nsec;
  uint8_t return_type;
  uint8_t label;
  uint8_t padding[9];

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(ca::NeaPoint,
  (double, x, x)
  (double, y, y)
  (double, z, z)
  (double, x_origin, x_origin)
  (double, y_origin, y_origin)
  (double, z_origin, z_origin)
  (float, range_variance, range_variance)
  (float, x_variance, x_variance)
  (float, y_variance, y_variance)
  (float, z_variance, z_variance)
  (float, reflectance, reflectance)
  (uint32_t, time_sec, time_sec)
  (uint32_t, time_nsec, time_nsec)
  (uint8_t, return_type, return_type)
  (uint8_t, label, label)
  (uint8_t[2], padding, padding))

POINT_CLOUD_REGISTER_POINT_STRUCT(ca::NeaPointf,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, x_origin, x_origin)
  (float, y_origin, y_origin)
  (float, z_origin, z_origin)
  (float, range_variance, range_variance)
  (float, x_variance, x_variance)
  (float, y_variance, y_variance)
  (float, z_variance, z_variance)
  (float, reflectance, reflectance)
  (uint32_t, time_sec, time_sec)
  (uint32_t, time_nsec, time_nsec)
  (uint8_t, return_type, return_type)
  (uint8_t, label, label)
  (uint8_t[9], padding, padding))

namespace ca
{

template<typename Scalar>
struct NeaPc2Metadata {
};

template<>
struct NeaPc2Metadata<double> {
  // Canonical NEA point cloud offsets.
  static const uint32_t x_offset              = 0;
  static const uint32_t y_offset              = 8;
  static const uint32_t z_offset              = 16;
  static const uint32_t x_origin_offset       = 24;
  static const uint32_t y_origin_offset       = 32;
  static const uint32_t z_origin_offset       = 40;
  static const uint32_t range_variance_offset = 48;
  static const uint32_t x_variance_offset     = 52;
  static const uint32_t y_variance_offset     = 56;
  static const uint32_t z_variance_offset     = 60;
  static const uint32_t reflectance_offset    = 64;
  static const uint32_t time_sec_offset       = 68;
  static const uint32_t time_nsec_offset      = 72;
  static const uint32_t return_type_offset    = 76;

  static const uint32_t point_length          = 77;
  static const uint32_t point_step            = 80;
  static const uint32_t padding               = 3;

  // unofficial extension: an extra byte for label
  // still fits in padding, point step is the same
  static const uint32_t label_offset          = 77;
  static const uint32_t point_length_l        = 78;
  static const uint32_t padding_l             = 2;

  // types
  static const uint32_t x_datatype              = sensor_msgs::PointField::FLOAT64;
  static const uint32_t y_datatype              = sensor_msgs::PointField::FLOAT64;
  static const uint32_t z_datatype              = sensor_msgs::PointField::FLOAT64;
  static const uint32_t x_origin_datatype       = sensor_msgs::PointField::FLOAT64;
  static const uint32_t y_origin_datatype       = sensor_msgs::PointField::FLOAT64;
  static const uint32_t z_origin_datatype       = sensor_msgs::PointField::FLOAT64;
  static const uint32_t range_variance_datatype = sensor_msgs::PointField::FLOAT32;
  static const uint32_t x_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t y_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t z_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t reflectance_datatype    = sensor_msgs::PointField::FLOAT32;
  static const uint32_t time_sec_datatype       = sensor_msgs::PointField::UINT32 ;
  static const uint32_t time_nsec_datatype      = sensor_msgs::PointField::UINT32 ;
  static const uint32_t return_type_datatype    = sensor_msgs::PointField::UINT8  ;
  static const uint32_t label_datatype          = sensor_msgs::PointField::UINT8  ;

};

template<>
struct NeaPc2Metadata<float> {
  // NEA point with single precision float point types
  static const uint32_t x_offset              = 0 ;
  static const uint32_t y_offset              = 4 ;
  static const uint32_t z_offset              = 8 ;
  static const uint32_t x_origin_offset       = 12;
  static const uint32_t y_origin_offset       = 16;
  static const uint32_t z_origin_offset       = 20;
  static const uint32_t range_variance_offset = 24;
  static const uint32_t x_variance_offset     = 28;
  static const uint32_t y_variance_offset     = 32;
  static const uint32_t z_variance_offset     = 36;
  static const uint32_t reflectance_offset    = 40;
  static const uint32_t time_sec_offset       = 44;
  static const uint32_t time_nsec_offset      = 48;
  static const uint32_t return_type_offset    = 52;

  static const uint32_t point_length          = 53;
  static const uint32_t point_step            = 64;
  static const uint32_t padding               = 10;
  // with label extension
  static const uint32_t label_offset          = 53;
  static const uint32_t point_length_l        = 54;
  static const uint32_t padding_l             = 9;

  // types
  static const uint32_t x_datatype              = sensor_msgs::PointField::FLOAT32;
  static const uint32_t y_datatype              = sensor_msgs::PointField::FLOAT32;
  static const uint32_t z_datatype              = sensor_msgs::PointField::FLOAT32;
  static const uint32_t x_origin_datatype       = sensor_msgs::PointField::FLOAT32;
  static const uint32_t y_origin_datatype       = sensor_msgs::PointField::FLOAT32;
  static const uint32_t z_origin_datatype       = sensor_msgs::PointField::FLOAT32;
  static const uint32_t range_variance_datatype = sensor_msgs::PointField::FLOAT32;
  static const uint32_t x_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t y_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t z_variance_datatype     = sensor_msgs::PointField::FLOAT32;
  static const uint32_t reflectance_datatype    = sensor_msgs::PointField::FLOAT32;
  static const uint32_t time_sec_datatype       = sensor_msgs::PointField::UINT32 ;
  static const uint32_t time_nsec_datatype      = sensor_msgs::PointField::UINT32 ;
  static const uint32_t return_type_datatype    = sensor_msgs::PointField::UINT8  ;
  static const uint32_t label_datatype          = sensor_msgs::PointField::UINT8  ;

};

template<class Scalar>
void GetNeaPc2Fields(std::vector<sensor_msgs::PointField>& fields, bool with_label=false) {
  using ca::pcl_util::make_pointfield;

  fields.clear(); // TODO necessary?
  fields.push_back(make_pointfield("x"              , NeaPc2Metadata<Scalar>::x_offset              , NeaPc2Metadata<Scalar>::x_datatype              , 1));
  fields.push_back(make_pointfield("y"              , NeaPc2Metadata<Scalar>::y_offset              , NeaPc2Metadata<Scalar>::y_datatype              , 1));
  fields.push_back(make_pointfield("z"              , NeaPc2Metadata<Scalar>::z_offset              , NeaPc2Metadata<Scalar>::z_datatype              , 1));
  fields.push_back(make_pointfield("x_origin"       , NeaPc2Metadata<Scalar>::x_origin_offset       , NeaPc2Metadata<Scalar>::x_origin_datatype       , 1));
  fields.push_back(make_pointfield("y_origin"       , NeaPc2Metadata<Scalar>::y_origin_offset       , NeaPc2Metadata<Scalar>::y_origin_datatype       , 1));
  fields.push_back(make_pointfield("z_origin"       , NeaPc2Metadata<Scalar>::z_origin_offset       , NeaPc2Metadata<Scalar>::z_origin_datatype       , 1));
  fields.push_back(make_pointfield("range_variance" , NeaPc2Metadata<Scalar>::range_variance_offset , NeaPc2Metadata<Scalar>::range_variance_datatype , 1));
  fields.push_back(make_pointfield("x_variance"     , NeaPc2Metadata<Scalar>::x_variance_offset     , NeaPc2Metadata<Scalar>::x_variance_datatype     , 1));
  fields.push_back(make_pointfield("y_variance"     , NeaPc2Metadata<Scalar>::y_variance_offset     , NeaPc2Metadata<Scalar>::y_variance_datatype     , 1));
  fields.push_back(make_pointfield("z_variance"     , NeaPc2Metadata<Scalar>::z_variance_offset     , NeaPc2Metadata<Scalar>::z_variance_datatype     , 1));
  fields.push_back(make_pointfield("reflectance"    , NeaPc2Metadata<Scalar>::reflectance_offset    , NeaPc2Metadata<Scalar>::reflectance_datatype    , 1));
  fields.push_back(make_pointfield("time_sec"       , NeaPc2Metadata<Scalar>::time_sec_offset       , NeaPc2Metadata<Scalar>::time_sec_datatype       , 1));
  fields.push_back(make_pointfield("time_nsec"      , NeaPc2Metadata<Scalar>::time_nsec_offset      , NeaPc2Metadata<Scalar>::time_nsec_datatype      , 1));
  fields.push_back(make_pointfield("return_type"    , NeaPc2Metadata<Scalar>::return_type_offset    , NeaPc2Metadata<Scalar>::return_type_datatype    , 1));
  if (with_label) {
    fields.push_back(make_pointfield("label"        , NeaPc2Metadata<Scalar>::label_offset          , NeaPc2Metadata<Scalar>::label_datatype          , 1));
  }
}

template<class Scalar>
uint32_t GetNeaPc2PointStep() {
  return NeaPc2Metadata<Scalar>::point_step;
}

template<class Scalar>
uint32_t GetNeaPc2PointLength(bool with_label=false) {
  return with_label ? NeaPc2Metadata<Scalar>::point_length_l : NeaPc2Metadata<Scalar>::point_length;
}

template<class Scalar>
uint32_t GetNeaPc2Padding(bool with_label=false) {
  return with_label ? NeaPc2Metadata<Scalar>::padding_l : NeaPc2Metadata<Scalar>::padding;
}

inline
size_t GetNeaPc2NumFields(bool with_label=false) {
  return with_label? 15 : 14;
}

#if 0
// TODO padding
bool CheckNeaPc2NumFields(const sensor_msgs::PointCloud2& cloud) {
  return (cloud.fields.size() == 14 || cloud.fields.size() == 15);
}
#endif

template<class Scalar>
inline bool CheckNeaPc2Datatype(const sensor_msgs::PointCloud2& cloud) {
  return false;
}

template<>
inline bool CheckNeaPc2Datatype<float>(const sensor_msgs::PointCloud2& cloud) {
  return ( cloud.fields[0].datatype==sensor_msgs::PointField::FLOAT32
           && cloud.fields[1].datatype==sensor_msgs::PointField::FLOAT32
           && cloud.fields[2].datatype==sensor_msgs::PointField::FLOAT32 );
}

template<>
inline bool CheckNeaPc2Datatype<double>(const sensor_msgs::PointCloud2& cloud) {
  return ( cloud.fields[0].datatype==sensor_msgs::PointField::FLOAT64
           && cloud.fields[1].datatype==sensor_msgs::PointField::FLOAT64
           && cloud.fields[2].datatype==sensor_msgs::PointField::FLOAT64 );
}

/**
 * Initialize fields and metadata to sensible values. Doesn't touch header or
 * data.
 */
template<class Scalar>
void InitializeNeaPc2(sensor_msgs::PointCloud2& pc,
                      bool with_label,
                      uint32_t num_points) {
  GetNeaPc2Fields<Scalar>(pc.fields, with_label);
  pc.point_step = GetNeaPc2PointStep<Scalar>();
  pc.height = 1;
  pc.width = num_points;
  pc.row_step = pc.point_step * pc.width;
  pc.is_dense = false; // not really used
}

template<class PointT>
void PclToPc2(const pcl::PointCloud<PointT>& pcin,
              sensor_msgs::PointCloud2& pcout) {
  pcl::PCLPointCloud2 pclpc;
  pcl::toPCLPointCloud2(pcin, pclpc);
  pcl_conversions::fromPCL(pclpc, pcout);
}

template<class PointT>
void Pc2ToPcl(const sensor_msgs::PointCloud2& pcin,
              pcl::PointCloud<PointT>& pcout) {
  pcl::PCLPointCloud2 tmppcl;
  pcl_conversions::toPCL(pcin, tmppcl);
  pcl::fromPCLPointCloud2(tmppcl, pcout);
}

} /* ca */

#endif /* end of include guard: NEA_PC_FORMAT_HPP_QTYJUJVB */
