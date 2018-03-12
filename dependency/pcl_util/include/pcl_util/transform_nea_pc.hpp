/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#ifndef TRANSFORM_NEA_PC_HPP_FIHO1SY5
#define TRANSFORM_NEA_PC_HPP_FIHO1SY5

#include <ros/console.h>

#include "nea_pc_format.hpp"

#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

namespace ca
{

// TODO should we also accept Affine3f?
template<class Scalar>
void TransformNeaPc2InPlace(const Eigen::Affine3d& cloud_to_world,
                            sensor_msgs::PointCloud2& cloud) {

  if (cloud.data.empty()) { return; }

  //ROS_ASSERT( CheckNeaPc2NumFields(cloud) );
  ROS_ASSERT( CheckNeaPc2Datatype<Scalar>(cloud) );
  ROS_ASSERT( cloud.point_step == NeaPc2Metadata<Scalar>::point_step );

  Eigen::Transform<Scalar, 3, Eigen::Affine> c2w(cloud_to_world.template cast<Scalar>());
  for (size_t i=0; i < cloud.width; ++i) {
    uint8_t *ptr = &(cloud.data[i * cloud.point_step]);
    Eigen::Map< Eigen::Matrix<Scalar, 3, 1> > p(reinterpret_cast<Scalar *>(ptr));
    p = c2w * p;

    Eigen::Map< Eigen::Matrix<Scalar, 3, 1> > vp(reinterpret_cast<Scalar *>(ptr + NeaPc2Metadata<Scalar>::x_origin_offset));
    vp = c2w * vp;
  }

}

inline
void TransformNeaPc2InPlace(const Eigen::Affine3d& cloud_to_world,
                            sensor_msgs::PointCloud2& cloud) {
  if (cloud.data.empty()) { return; }
  if (cloud.fields[0].datatype==sensor_msgs::PointField::FLOAT32) {
    TransformNeaPc2InPlace<float>(cloud_to_world, cloud);
  } else if (cloud.fields[0].datatype==sensor_msgs::PointField::FLOAT64) {
    TransformNeaPc2InPlace<double>(cloud_to_world, cloud);
  } else {
    ROS_ERROR("Unknown point cloud type");
  }
}

} /* ca */

#endif /* end of include guard: TRANSFORM_NEA_PC_HPP_FIHO1SY5 */
