/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/



#include <pcl_util/pcl_util.hpp>
#include <pcl_util/nea_pc_format.hpp>
#include <pcl_util/transform_nea_pc.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <gtest/gtest.h>

TEST(TransformNeaPcInPlace, testdd) {
  using namespace ca;
  using namespace Eigen;

  Vector3d ep(2, 8, 10);
  Vector3d evp(10, 2, 8);

  pcl::PointCloud<NeaPoint> tmppc;
  NeaPoint p;
  p.x = ep.x(); p.y = ep.y(); p.z = ep.z();
  p.x_origin = evp.x(); p.y_origin = evp.y(); p.z_origin = evp.z();
  tmppc.push_back(p);
  tmppc.push_back(p);

  sensor_msgs::PointCloud2 pc2;
  ca::PclToPc2(tmppc, pc2);

  Affine3d etf = Translation3d(Vector3d(2, 2, 2)) * Quaterniond::Identity();
  TransformNeaPc2InPlace<double>(etf, pc2);

  pcl::PointCloud<NeaPoint> tmppc_out;
  Pc2ToPcl(pc2, tmppc_out);

  for (size_t i=0; i < tmppc_out.size(); ++i) {
    EXPECT_EQ(tmppc_out[i].x, 4);
    EXPECT_EQ(tmppc_out[i].y, 10);
    EXPECT_EQ(tmppc_out[i].z, 12);

    EXPECT_EQ(tmppc_out[i].x_origin, 12);
    EXPECT_EQ(tmppc_out[i].y_origin, 4);
    EXPECT_EQ(tmppc_out[i].z_origin, 10);
  }

}

TEST(TransformNeaPcInPlace, testfd) {
  using namespace ca;
  using namespace Eigen;

  Vector3f ep(2, 8, 10);
  Vector3f evp(10, 2, 8);

  pcl::PointCloud<NeaPointf> tmppc;
  NeaPointf p;
  p.x = ep.x(); p.y = ep.y(); p.z = ep.z();
  p.x_origin = evp.x(); p.y_origin = evp.y(); p.z_origin = evp.z();
  tmppc.push_back(p);
  tmppc.push_back(p);

  sensor_msgs::PointCloud2 pc2;
  ca::PclToPc2(tmppc, pc2);

  Affine3d etf = Translation3d(Vector3d(2, 2, 2)) * Quaterniond::Identity();
  TransformNeaPc2InPlace<float>(etf, pc2);

  pcl::PointCloud<NeaPointf> tmppc_out;
  Pc2ToPcl(pc2, tmppc_out);

  for (size_t i=0; i < tmppc_out.size(); ++i) {
    EXPECT_EQ(tmppc_out[i].x, 4);
    EXPECT_EQ(tmppc_out[i].y, 10);
    EXPECT_EQ(tmppc_out[i].z, 12);

    EXPECT_EQ(tmppc_out[i].x_origin, 12);
    EXPECT_EQ(tmppc_out[i].y_origin, 4);
    EXPECT_EQ(tmppc_out[i].z_origin, 10);
  }

}

TEST(TransformNeaPcInPlace, testdyn1) {
  using namespace ca;
  using namespace Eigen;

  Vector3f ep(2, 8, 10);
  Vector3f evp(10, 2, 8);

  pcl::PointCloud<NeaPointf> tmppc;
  NeaPointf p;
  p.x = ep.x(); p.y = ep.y(); p.z = ep.z();
  p.x_origin = evp.x(); p.y_origin = evp.y(); p.z_origin = evp.z();
  tmppc.push_back(p);
  tmppc.push_back(p);

  sensor_msgs::PointCloud2 pc2;
  ca::PclToPc2(tmppc, pc2);

  Affine3d etf = Translation3d(Vector3d(2, 2, 2)) * Quaterniond::Identity();
  TransformNeaPc2InPlace(etf, pc2);

  pcl::PointCloud<NeaPointf> tmppc_out;
  Pc2ToPcl(pc2, tmppc_out);

  for (size_t i=0; i < tmppc_out.size(); ++i) {
    EXPECT_EQ(tmppc_out[i].x, 4);
    EXPECT_EQ(tmppc_out[i].y, 10);
    EXPECT_EQ(tmppc_out[i].z, 12);

    EXPECT_EQ(tmppc_out[i].x_origin, 12);
    EXPECT_EQ(tmppc_out[i].y_origin, 4);
    EXPECT_EQ(tmppc_out[i].z_origin, 10);
  }

}

TEST(TransformNeaPcInPlace, testdyn2) {
  using namespace ca;
  using namespace Eigen;

  Vector3d ep(2, 8, 10);
  Vector3d evp(10, 2, 8);

  pcl::PointCloud<NeaPoint> tmppc;
  NeaPoint p;
  p.x = ep.x(); p.y = ep.y(); p.z = ep.z();
  p.x_origin = evp.x(); p.y_origin = evp.y(); p.z_origin = evp.z();
  tmppc.push_back(p);
  tmppc.push_back(p);

  sensor_msgs::PointCloud2 pc2;
  ca::PclToPc2(tmppc, pc2);

  Affine3d etf = Translation3d(Vector3d(2, 2, 2)) * Quaterniond::Identity();
  TransformNeaPc2InPlace(etf, pc2);

  pcl::PointCloud<NeaPoint> tmppc_out;
  Pc2ToPcl(pc2, tmppc_out);

  for (size_t i=0; i < tmppc_out.size(); ++i) {
    EXPECT_EQ(tmppc_out[i].x, 4);
    EXPECT_EQ(tmppc_out[i].y, 10);
    EXPECT_EQ(tmppc_out[i].z, 12);

    EXPECT_EQ(tmppc_out[i].x_origin, 12);
    EXPECT_EQ(tmppc_out[i].y_origin, 4);
    EXPECT_EQ(tmppc_out[i].z_origin, 10);
  }

}


TEST(NeaPoint, size1) {
  using namespace ca;
  NeaPoint pt;
  EXPECT_EQ(sizeof(pt), GetNeaPc2PointStep<double>());

  NeaPointf pt2;
  EXPECT_EQ(sizeof(pt2), GetNeaPc2PointStep<float>());
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  return 0;
}
