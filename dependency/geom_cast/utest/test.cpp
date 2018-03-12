/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <Eigen/Core>

#include <geom_cast/geom_cast.hpp>

#include <gtest/gtest.h>

using namespace boost::assign;
using namespace Eigen;
using namespace ca;

TEST(geom_cast, points_3_2) {
  Eigen::Vector3d v1(2, 8, 1);
  EXPECT_FLOAT_EQ( v1.x(), 2. );
  EXPECT_FLOAT_EQ( v1.y(), 8. );

  pcl::PointXY v2 = ca::point_cast<pcl::PointXY>(v1);

  EXPECT_FLOAT_EQ( v2.x, 2. );
  EXPECT_FLOAT_EQ( v2.y, 8. );

  pcl::PointXYZ v3(2, 8, 1);
  pcl::PointXY v4 = ca::point_cast<pcl::PointXY>(v3);
  EXPECT_FLOAT_EQ( v4.x, 2. );
  EXPECT_FLOAT_EQ( v4.y, 8. );

  cv::Vec2d v5 = ca::point_cast<cv::Vec2d>(v1);

  EXPECT_FLOAT_EQ( v5[0], 2. );
  EXPECT_FLOAT_EQ( v5[1], 8. );
}

TEST(geom_cast, points3) {

  Eigen::Vector3d food(2, 8, 0);
  Eigen::Vector3f foof(2, 8, 0);

  tf::Vector3 v1 = ca::point_cast<tf::Vector3>(food);

  EXPECT_FLOAT_EQ( v1.x(), 2. );
  EXPECT_FLOAT_EQ( v1.y(), 8. );
  EXPECT_FLOAT_EQ( v1.z(), 0. );

  tf::Vector3 v2 = ca::point_cast<tf::Vector3>(foof);

  EXPECT_FLOAT_EQ( v2.x(), 2. );
  EXPECT_FLOAT_EQ( v2.y(), 8. );
  EXPECT_FLOAT_EQ( v2.z(), 0. );

  geometry_msgs::Point v3 = ca::point_cast<geometry_msgs::Point>(foof);

  EXPECT_FLOAT_EQ( v3.x,  2.);
  EXPECT_FLOAT_EQ( v3.y,  8.);
  EXPECT_FLOAT_EQ( v3.z,  0.);

  geometry_msgs::Point v4 = ca::point_cast<geometry_msgs::Point>(v2);

  EXPECT_FLOAT_EQ( v4.x, 2.);
  EXPECT_FLOAT_EQ( v4.y, 8.);
  EXPECT_FLOAT_EQ( v4.z, 0.);

  pcl::PointXYZ v5 = ca::point_cast<pcl::PointXYZ>(foof);

  EXPECT_FLOAT_EQ( v5.x,  2.);
  EXPECT_FLOAT_EQ( v5.y,  8.);
  EXPECT_FLOAT_EQ( v5.z,  0.);

  Eigen::Vector3d v6 = ca::point_cast<Eigen::Vector3d>(v5);

  EXPECT_FLOAT_EQ( v6.x(), 2. );
  EXPECT_FLOAT_EQ( v6.y(), 8. );
  EXPECT_FLOAT_EQ( v6.z(), 0. );

  Eigen::Vector3d v7 = ca::point_cast<Eigen::Vector3d>(v1);

  EXPECT_FLOAT_EQ( v7.x(), 2. );
  EXPECT_FLOAT_EQ( v7.y(), 8. );
  EXPECT_FLOAT_EQ( v7.z(), 0. );

  float * v8 = static_cast<float*>(malloc(sizeof(float)*3));
  v8[0] = 2; v8[1] = 8; v8[2] = 0;
  pcl::PointXYZ v9 = ca::point_cast<pcl::PointXYZ>(v8);

  EXPECT_FLOAT_EQ( v7.x(),  2.);
  EXPECT_FLOAT_EQ( v7.y(),  8.);
  EXPECT_FLOAT_EQ( v7.z(),  0.);

  float v10[3] = { 2., 8., 0.};
  pcl::PointXYZ v11 = ca::point_cast<pcl::PointXYZ>(v10);
  EXPECT_FLOAT_EQ( v11.x,  2.);
  EXPECT_FLOAT_EQ( v11.y,  8.);
  EXPECT_FLOAT_EQ( v11.z,  0.);


  cv::Point3_<float> v12 = ca::point_cast<cv::Point3_<float> >(v6);
  EXPECT_FLOAT_EQ( v12.x,  2.);
  EXPECT_FLOAT_EQ( v12.y,  8.);
  EXPECT_FLOAT_EQ( v12.z,  0.);

  cv::Vec<double, 3> v13 = ca::point_cast<cv::Vec3d>(v4);
  EXPECT_FLOAT_EQ( v13[0],  2.);
  EXPECT_FLOAT_EQ( v13[1],  8.);
  EXPECT_FLOAT_EQ( v13[2],  0.);
}

TEST(geom_cast, rotations) {
  // eigen ctor is wxyz
  Eigen::Quaterniond q1(1, 0, 0, 0);
  EXPECT_FLOAT_EQ(q1.x(), 0.);
  EXPECT_FLOAT_EQ(q1.y(), 0.);
  EXPECT_FLOAT_EQ(q1.z(), 0.);
  EXPECT_FLOAT_EQ(q1.w(), 1.);

  geometry_msgs::Quaternion q2 = ca::rot_cast<geometry_msgs::Quaternion>(q1);
  EXPECT_FLOAT_EQ(q2.x, 0.);
  EXPECT_FLOAT_EQ(q2.y, 0.);
  EXPECT_FLOAT_EQ(q2.z, 0.);
  EXPECT_FLOAT_EQ(q2.w, 1.);

  tf::Quaternion q3 = ca::rot_cast<tf::Quaternion>(q1);

  EXPECT_FLOAT_EQ(q3.x(), 0.);
  EXPECT_FLOAT_EQ(q3.y(), 0.);
  EXPECT_FLOAT_EQ(q3.z(), 0.);
  EXPECT_FLOAT_EQ(q3.w(), 1.);

  Eigen::Quaternionf q4 = ca::rot_cast<Eigen::Quaternionf>(q3);

  EXPECT_FLOAT_EQ(q4.x(), 0.);
  EXPECT_FLOAT_EQ(q4.y(), 0.);
  EXPECT_FLOAT_EQ(q4.z(), 0.);
  EXPECT_FLOAT_EQ(q4.w(), 1.);

}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
