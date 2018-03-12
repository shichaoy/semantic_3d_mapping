/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#ifndef PCL_UTIL_HPP_MWOLDV6B
#define PCL_UTIL_HPP_MWOLDV6B

#include <string>
#include <vector>
#include <numeric>
#include <iostream>
#include <algorithm>

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_traits.h>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_types.hpp"

namespace ca
{
namespace pcl_util
{

struct PointExtentStats {
 public:
  pcl::PointXYZ min_pt; ///< min xyz
  pcl::PointXYZ max_pt; ///< max xyz
  pcl::PointXYZ q05_pt; ///< 5th percentile xyz
  pcl::PointXYZ q95_pt; ///< 95th percentile xyz
  pcl::PointXYZ mean_pt; ///< mean xyz
  pcl::PointXYZ med_pt; ///< median xyz
};

/***
 * Get the index of a pointfield type in a pointcloud2
 * MAKE SURE THE POINTFIELD EXISTS OR THIS WILL FAIL UNPREDICTABLY
 *
 * @param pc2 input cloud
 * @param field_name name of the field
 */
inline
uint32_t get_field_offset(const PC2& pc2, const char* field_name) {
  ROS_ASSERT( pcl::getFieldIndex(pc2, field_name) >= 0 );
  return pc2.fields[pcl::getFieldIndex(pc2, field_name)].offset;
}

/**
 * get offsets of x y z
 * don't use these unless you are sure xyz are not adjacent
 */
inline
void get_xyz_offsets(const PC2& pc2,
                     uint32_t &x_offset,
                     uint32_t &y_offset,
                     uint32_t &z_offset) {
  x_offset = pc2.fields[pcl::getFieldIndex(pc2, "x")].offset;
  y_offset = pc2.fields[pcl::getFieldIndex(pc2, "y")].offset;
  z_offset = pc2.fields[pcl::getFieldIndex(pc2, "z")].offset;
}

/**
 * get index offsets of [x,y,z]_origin
 */
inline
void get_xyz_origin_offsets(const PC2& pc2,
                            uint32_t &x_orig_offset,
                            uint32_t &y_orig_offset,
                            uint32_t &z_orig_offset) {
  x_orig_offset = pc2.fields[pcl::getFieldIndex(pc2, "x_origin")].offset;
  y_orig_offset = pc2.fields[pcl::getFieldIndex(pc2, "y_origin")].offset;
  z_orig_offset = pc2.fields[pcl::getFieldIndex(pc2, "z_origin")].offset;
}

inline
void get_xy_offsets(const PC2& pc2,
                    uint32_t &x_offset,
                    uint32_t &y_offset) {
  x_offset = pc2.fields[pcl::getFieldIndex(pc2, "x")].offset;
  y_offset = pc2.fields[pcl::getFieldIndex(pc2, "y")].offset;
}

/**
 * Convert a vector of Eigen column vectors into a point cloud.
 * All vectors must have same size and type.
 *
 * @param colvecs the vector of vectors, where each vector is a "column".
 * @param colnames names of columns
 * @param out output point cloud.
 */
template<class Scalar>
void vecs_to_point_cloud(const std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1> >& colvecs,
                         const std::vector<std::string>& colnames,
                         PC2::Ptr out) {
  ROS_ASSERT(!colnames.empty());
  ROS_ASSERT(!colvecs.empty());
  ROS_ASSERT(colvecs.size()==colnames.size());

  //check dimensions
  size_t D = colnames.size(), N = colvecs[0].size();
  for (size_t i=0, sz=colvecs.size(); i<sz; ++i) {
    if (static_cast<size_t>(colvecs[i].size())!=N) {
      throw std::runtime_error("columns have unequal number of items");
    }
  }

  // now do business
  out->width = N;
  out->height = 1;
  size_t data_size = sizeof(Scalar)*N*D;
  size_t point_step = sizeof(Scalar)*D;
  out->data.clear();
  out->data.resize(data_size);
  //std::fill(out->data.begin(), out->data.end(), Scalar(0));
  // alternative: use eigen matrices?
  for (size_t i=0; i < N; ++i) {
    Scalar *row = reinterpret_cast<Scalar*>(&(out->data[0])+i*D*sizeof(Scalar));
    for (size_t j = 0; j < D; ++j) {
      row[j] = colvecs[j](i);
    }
  }

  // fill metadata
  out->fields.clear();
  for (size_t j=0; j < D; ++j) {
    sensor_msgs::PointField f;
    //pcl::PCLPointField f;
    f.name = colnames.at(j);
    f.offset = sizeof(Scalar)*j;
    f.datatype = pcl::traits::asEnum<Scalar>::value;
    f.count = 1;
    out->fields.push_back(f);
  }

  //out->header = std_msgs::Header();
  //out->header.seq = 0;
  //out->header.stamp = 0;
  //out->header.frame_id = 0;
  out->point_step = point_step;
  out->row_step = data_size;
  //out->is_dense = true;
  //out->is_bigendian?
}

/**
 * convert a pointcloud2 to a cv::Mat
 */
template<class Scalar>
void pc2_to_cv_mat(const PC2::Ptr feat_pc,
                   cv::Mat_<Scalar>& feat_mat) {
  uint32_t N = feat_pc->width;
  int D = 0;
  for (size_t i=0; i < feat_pc->fields.size(); ++i) {
    D += feat_pc->fields[i].count;
  }
  // feat_pc->point_step should be D*sizeof(float)
  feat_mat = cv::Mat_<Scalar>(N, D, feat_pc->data.data(), feat_pc->point_step);
}

#if 0
/**
 * concatenate multiple point clouds "sideways"
 * note that the pointfields shouldn't overlap
 * note that this is inefficient since it involves a lot of recopying.
 */
inline
void concatenate_multiple_fields(std::vector<PC2::Ptr>& pcs, ca::PC2& out) {
  PC2 all;
  for (size_t i=0; i < pcs.size(); ++i) {
    PC2 tmp;
    pcl::concatenateFields( *(pcs[i]), all, tmp );
    std::swap(tmp, all);
  }
  out = all;
}
#endif

/**
 * concatenate multiple point clouds "sideways"
 * note that the pointfields shouldn't overlap
 * also sizes should be equal
 * TODO needs testing
 */
inline
void concatenate_multiple_fields(std::vector<PC2::Ptr>& pcs, ca::PC2& out) {
  // TODO this ignores the issue of padding

  if (pcs.empty()) {
    out.fields.clear();
    out.data.clear();
    out.width = out.height = out.row_step = out.point_step = 0;
    return;
  }

  out.width = pcs[0]->width;
  out.height  = 1;
  for ( size_t i=0; i < pcs.size(); ++i) {
    ROS_ASSERT(pcs[i]->width == out.width);
  }

  out.point_step = 0;
  out.row_step = 0;
  for ( size_t i=0; i < pcs.size(); ++i) {
    out.point_step += pcs[i]->point_step;
    out.row_step += pcs[i]->row_step;
  }

  out.fields.clear();
  for ( size_t i=0; i < pcs.size(); ++i) {
    out.fields.insert(out.fields.end(), pcs[i]->fields.begin(), pcs[i]->fields.end());
  }

  std::vector<uint32_t> new_offsets;
  uint32_t cum_offset = 0;
  for ( size_t i=0; i < pcs.size(); ++i) {
    for ( size_t j=0; j < pcs[i]->fields.size(); ++j) {
      new_offsets.push_back(pcs[i]->fields[j].offset + cum_offset);
    }
    cum_offset += pcs[i]->point_step;
  }

  ROS_ASSERT(new_offsets.size() == out.fields.size());
  for ( size_t i=0; i < out.fields.size(); ++i) {
    out.fields[i].offset = new_offsets[i];
  }

  ROS_ASSERT(out.row_step == out.point_step*out.width);
  out.data.reserve(out.row_step);

  for ( size_t i=0; i < out.width; ++i ) {

    std::vector<uint8_t>::iterator out_row_itr( out.data.begin() + i*out.point_step );

    for ( size_t j=0; j < pcs.size(); ++j) {

      std::vector<uint8_t>::iterator row_begin(pcs[j]->data.begin() + i*pcs[j]->point_step);
      std::vector<uint8_t>::iterator row_end(row_begin + pcs[j]->point_step);

      out.data.insert(out_row_itr, row_begin, row_end);
      out_row_itr += pcs[j]->point_step;

    }
  }
}

/**
 * concatenate 'downwards'. fields must be the same.
 */
inline
void concatenate_multiple_clouds(const std::vector<sensor_msgs::PointCloud2>& clouds,
                                 sensor_msgs::PointCloud2& out_pc) {
  size_t total_points = 0;
  BOOST_FOREACH(const sensor_msgs::PointCloud2& pci, clouds) {
    total_points += pci.width;
  }

  out_pc.data.clear();
  out_pc.fields.clear();
  out_pc.width = out_pc.height = 0;
  out_pc.point_step = out_pc.row_step = 0;

  if (clouds.empty() || total_points == 0) {
    return;
  }

  for ( size_t i=0; i < clouds.size(); ++i ) {
    ROS_ASSERT(clouds[0].fields.size()==clouds[i].fields.size());
  }

  out_pc.data.reserve(clouds.front().point_step*total_points);
  BOOST_FOREACH(const sensor_msgs::PointCloud2& pci, clouds) {
    out_pc.width += pci.width;
    out_pc.data.insert(out_pc.data.end(), pci.data.begin(), pci.data.end());
  }
  out_pc.header = clouds.front().header;
  out_pc.fields = clouds.front().fields;
  out_pc.point_step = clouds.front().point_step;
  out_pc.height = 1;
  out_pc.row_step = out_pc.point_step*out_pc.width;
  out_pc.is_dense = false;
}

/**
 * find trimmed extrema of seq. sorts in place.
 */
template<class T>
void destructive_trimmed_extrema(std::vector<T>& seq,
                                 T& min_v,
                                 T& max_v) {
  std::sort(seq.begin(), seq.end());
  min_v = seq.at(static_cast<size_t>(0.05*seq.size()));
  max_v = seq.at(static_cast<size_t>(0.95*seq.size()));
}

/**
 * @param r float/double in 0..1 range
 * @param g float/double in 0..1 range
 * @param b float/double in 0..1 range
 * @return rgb packed into float
 */
template<class T>
float real_to_pcl_rgb(T r, T g, T b) {
  BOOST_STATIC_ASSERT_MSG(boost::is_floating_point<T>::value, "Only floating point");
  uint32_t ri = static_cast<uint32_t>(r*255);
  uint32_t gi = static_cast<uint32_t>(g*255);
  uint32_t bi = static_cast<uint32_t>(b*255);
  uint32_t rgb = (ri<<16)|(gi<<8)|bi;
  //return (*reinterpret_cast<float*>(&rgb));
  union { float as_float; uint32_t as_uint32; } value;
  value.as_uint32 = rgb;
  return value.as_float;
}

/**
 * @param r (u)int in 0..255 range
 * @param g (u)int in 0..255 range
 * @param b (u)int in 0..255 range
 * @return rgb packed into float
 */
template<class T>
float int_to_pcl_rgb(T r, T g, T b) {
  BOOST_STATIC_ASSERT_MSG(boost::is_integral<T>::value, "Only integer types");
  // TODO clip before cast?
  uint32_t ri = static_cast<uint32_t>(r);
  uint32_t gi = static_cast<uint32_t>(g);
  uint32_t bi = static_cast<uint32_t>(b);
  uint32_t rgb = (ri<<16)|(gi<<8)|bi;
  //return (*reinterpret_cast<float*>(&rgb));
  union { float as_float; uint32_t as_uint32; } value;
  value.as_uint32 = rgb;
  return value.as_float;
}

inline
boost::tuple<uint8_t, uint8_t, uint8_t>
pcl_rgb_to_int(float rgb) {
  //uint32_t irgb = *reinterpret_cast<uint32_t*>(&rgb);
  union { float as_float; uint32_t as_uint32; } value;
  value.as_float = rgb;
  uint32_t irgb = value.as_uint32;
  uint8_t r = (irgb >> 16) & 0x0000ff;
  uint8_t g = (irgb >> 8) & 0x0000ff;
  uint8_t b = (irgb) & 0x0000ff;
  return boost::tuple<uint8_t, uint8_t, uint8_t>(r, g, b);
}

/**
 * Pretty much a constructor for sensor_msgs::pointfield.
 */
inline
sensor_msgs::PointField make_pointfield(const std::string& name,
                                        uint32_t offset,
                                        uint32_t datatype,
                                        uint32_t count) {
  sensor_msgs::PointField pf;
  pf.name = name;
  pf.offset = offset;
  pf.datatype = datatype;
  pf.count = count;
  return pf;
}

/**
 * Statistics about the spatial extent of a point cloud.
 * complexity is n log n
 */
template<class Scalar>
void point_cloud_extent_stats(const PC2& input_pc,
                              PointExtentStats& pexstats) {

  uint32_t x_offset, y_offset, z_offset;
  get_xyz_offsets(input_pc, x_offset, y_offset, z_offset);
  size_t num_points = input_pc.width;
  std::vector<Scalar> x(num_points), y(num_points), z(num_points);
  for (size_t i=0; i < num_points; ++i) {
    const uint8_t* pt = &(input_pc.data[i*input_pc.point_step]);
    x[i] = *reinterpret_cast<const Scalar*>(pt + x_offset);
    y[i] = *reinterpret_cast<const Scalar*>(pt + y_offset);
    z[i] = *reinterpret_cast<const Scalar*>(pt + z_offset);
  }

  // TODO sample instead of sorting everything
  std::sort(x.begin(), x.end());
  std::sort(y.begin(), y.end());
  std::sort(z.begin(), z.end());

  pexstats.min_pt = pcl::PointXYZ(x.front(), y.front(), z.front());
  pexstats.max_pt = pcl::PointXYZ(x.back(), y.back(), z.back());

  size_t ix05 = static_cast<size_t>(.05*num_points);
  size_t ix95 = static_cast<size_t>(.95*num_points);
  size_t ix50 = static_cast<size_t>(.5*num_points);
  pexstats.q05_pt = pcl::PointXYZ(x[ix05], y[ix05], z[ix05]);
  pexstats.q95_pt = pcl::PointXYZ(x[ix95], y[ix95], z[ix95]);
  pexstats.med_pt = pcl::PointXYZ(x[ix50], y[ix50], z[ix50]);

  Scalar x_sum = std::accumulate(x.begin(), x.end(), 0);
  pexstats.mean_pt.x = x_sum/num_points;

  Scalar y_sum = std::accumulate(y.begin(), y.end(), 0);
  pexstats.mean_pt.y = y_sum/num_points;

  Scalar z_sum = std::accumulate(z.begin(), z.end(), 0);
  pexstats.mean_pt.z = z_sum/num_points;
}

template<class Scalar>
void compute_origin_mean(const PC2& cloud,
                         Eigen::Matrix<Scalar, 3, 1>& origin,
                         int every_nth=1) {
  static uint32_t x_origin_offset = get_field_offset(cloud, "x_origin");
  //TODO verify compatibility using cloud.field type
  //TODO numerical stability?

  // first get the approx sensor position using origin field
  origin.setZero();
  // just approximating by subsampling
  for (const uint8_t *ptr = &(cloud.data[0]),
       *end_ptr = (&(cloud.data[0])+cloud.row_step);
       ptr < end_ptr;
       ptr += (every_nth*cloud.point_step) ) {
    Eigen::Map<const Eigen::Matrix<Scalar, 3, 1> > p(reinterpret_cast<const Scalar*>(ptr+x_origin_offset));
    origin += p;
  }
  origin = (every_nth*origin)/cloud.width;

}

inline
void check_pointcloud2_sanity(const ca::PC2& cloud) {
  ROS_ASSERT( cloud.height == 1 );
  ROS_ASSERT( cloud.point_step != 0 );
  ROS_ASSERT( cloud.row_step == cloud.point_step * cloud.width );
  ROS_ASSERT( cloud.data.size() == cloud.point_step * cloud.width * cloud.height );
  ROS_ASSERT( !cloud.fields.empty() );

  // offsets must be increasing
  if (cloud.fields.size() > 1) {
    for (size_t i=1; i < cloud.fields.size(); ++i) {
      ROS_ASSERT(cloud.fields[i].offset > cloud.fields[i-1].offset);
    }
  }

  // nonzero counts
  for (size_t i=0; i < cloud.fields.size(); ++i) {
    ROS_ASSERT(cloud.fields[i].count != 0);
  }

  ROS_ASSERT(cloud.fields.back().offset < cloud.point_step);

  ROS_ASSERT(!cloud.header.frame_id.empty());
  // TODO more checks needed
}

// why doesn't have pointxy have a ctor???
template<typename Scalar>
pcl::PointXY make_pcl_xy(Scalar x, Scalar y) {
  pcl::PointXY p;
  p.x = x;
  p.y = y;
  return p;
}

template<typename Scalar>
pcl::PointXYZ make_pcl_xyz(Scalar x, Scalar y, Scalar z) {
  return pcl::PointXYZ(x, y, z);
}

// for geometry_msg/point or pcl xyz
template<typename PointT, typename Real>
PointT make_point_xyz(Real x, Real y, Real z) {
  PointT pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  return pt;
}

template<typename Real>
pcl::PointXYZI make_pcl_xyzi(Real x, Real y, Real z, Real intensity) {
  pcl::PointXYZI pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  pt.intensity = intensity;
  return pt;
}

inline
void keep_points(sensor_msgs::PointCloud2& cloud, const std::vector<uint32_t>& indices) {
  if (cloud.width==0) {
    ROS_WARN("keep_points input has no points");
    return;
  }
  ROS_ASSERT( indices.size() <= cloud.width );

  // TODO check for duplicates?
  if (indices.size() == cloud.width) { // nothing to be done
    return;
  }

  //uint32_t pre_width = cloud.width;
  std::vector<uint8_t> new_data;
  new_data.reserve( indices.size()*cloud.point_step );
  BOOST_FOREACH(uint32_t ix, indices) {
    const uint8_t *ptr = &(cloud.data[0]) + ix*cloud.point_step;
    // TODO is some form of emplace better?
    new_data.insert(new_data.end(), ptr, (ptr+cloud.point_step));
  }

  cloud.data.swap(new_data);
  cloud.width = indices.size();
  cloud.row_step = cloud.point_step*indices.size();

  //ROS_INFO("keep_points: %u input, %u output", pre_width, cloud->width);
}

// NOTE don't use this - it's just a sanity check
inline
Eigen::Vector3f get_centroid(const sensor_msgs::PointCloud2& cloud) {
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (size_t i=0; i < cloud.width; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step]);
    float y = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + sizeof(float)]);
    float z = *reinterpret_cast<const float*>(&cloud.data[i*cloud.point_step + 2*sizeof(float)]);
    center.x() += x;
    center.y() += y;
    center.z() += z;
  }
  center /= cloud.width;
  return center;
}

}
} /* ca */

#endif /* end of include guard: PCL_UTIL_HPP_MWOLDV6B */
