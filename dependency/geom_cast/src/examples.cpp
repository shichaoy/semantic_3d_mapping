/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#include <stdlib.h>
#include <iostream>
#include "geom_cast/point_cast.hpp"
#include "geom_cast/rot_cast.hpp"

void points32() {
  std::cerr << "Each line should be 2, 8\n";

  Eigen::Vector3d v1(2, 8, 1);
  std::cerr << v1.x() << ", " << v1.y() << "\n";

  pcl::PointXY v2 = ca::point_cast<pcl::PointXY>(v1);
  std::cerr << v2.x << ", " << v2.y << "\n";

  pcl::PointXYZ v3(2, 8, 1);
  pcl::PointXY v4 = ca::point_cast<pcl::PointXY>(v3);
  std::cerr << v4.x << ", " << v4.y << "\n";

  cv::Vec2d v5 = ca::point_cast<cv::Vec2d>(v1);
  std::cerr << v5[0] << ", " << v5[1] << "\n";
}

void points3() {

  std::cerr << "Each line should be 2, 8, 0\n";
  Eigen::Vector3d food(2, 8, 0);
  Eigen::Vector3f foof(2, 8, 0);

  tf::Vector3 v1 = ca::point_cast<tf::Vector3>(food);

  std::cerr << v1.x() << ", " << v1.y() << ", " << v1.z() << "\n";

  tf::Vector3 v2 = ca::point_cast<tf::Vector3>(foof);

  std::cerr << v2.x() << ", " << v2.y() << ", " << v2.z() << "\n";

  geometry_msgs::Point v3 = ca::point_cast<geometry_msgs::Point>(foof);

  std::cerr << v3.x << ", " << v3.y << ", " << v3.z << "\n";

  geometry_msgs::Point v4 = ca::point_cast<geometry_msgs::Point>(v2);

  std::cerr << v4.x << ", " << v4.y << ", " << v4.z << "\n";

  pcl::PointXYZ v5 = ca::point_cast<pcl::PointXYZ>(foof);

  std::cerr << v5.x << ", " << v5.y << ", " << v5.z << "\n";

  Eigen::Vector3d v6 = ca::point_cast<Eigen::Vector3d>(v5);

  std::cerr << v6.x() << ", " << v6.y() << ", " << v6.z() << "\n";

  Eigen::Vector3d v7 = ca::point_cast<Eigen::Vector3d>(v1);

  std::cerr << v7.x() << ", " << v7.y() << ", " << v7.z() << "\n";

  float * v8 = static_cast<float*>(malloc(sizeof(float)*3));
  v8[0] = 2; v8[1] = 8; v8[2] = 0;
  pcl::PointXYZ v9 = ca::point_cast<pcl::PointXYZ>(v8);

  std::cerr << v9.x << ", " << v9.y << ", " << v9.z << "\n";

  float v10[3] = { 2., 8., 0.};
  pcl::PointXYZ v11 = ca::point_cast<pcl::PointXYZ>(v10);

  std::cerr << v11.x << ", " << v11.y << ", " << v11.z << "\n";

  cv::Point3_<float> v12 = ca::point_cast<cv::Point3_<float> >(v6);

  std::cerr << v12.x << ", " << v12.y << ", " << v12.z << "\n";

  cv::Vec<double, 3> v13 = ca::point_cast<cv::Vec3d>(v4);
  std::cerr << v13[0] << ", " << v13[1] << ", " << v13[2] << "\n";
}

void rotations() {
  std::cerr << "Each line should be 0, 0, 0, 1\n";
  // eigen ctor is wxyz
  Eigen::Quaterniond q1(1, 0, 0, 0);
  std::cerr << q1.x() << ", "<< q1.y() << ", "<< q1.z() << ", "<< q1.w() << "\n";

  geometry_msgs::Quaternion q2 = ca::rot_cast<geometry_msgs::Quaternion>(q1);
  std::cerr << q2.x << ", "<< q2.y << ", "<< q2.z << ", "<< q2.w << "\n";

  tf::Quaternion q3 = ca::rot_cast<tf::Quaternion>(q1);
  std::cerr << q3.x() << ", "<< q3.y() << ", "<< q3.z() << ", "<< q3.w() << "\n";

  Eigen::Quaternionf q4 = ca::rot_cast<Eigen::Quaternionf>(q3);
  std::cerr << q4.x() << ", "<< q4.y() << ", "<< q4.z() << ", "<< q4.w() << "\n";

}

int main(int argc, char *argv[]) {

  points3();
  std::cerr << "\n";
  rotations();
  std::cerr << "\n";
  points32();
  return 0;
}
