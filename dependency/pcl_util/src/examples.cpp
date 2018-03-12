/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/


#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_util/point_types.hpp>
#include <pcl_util/pcl_util.hpp>
#include <pcl_util/nea_pc_format.hpp>

int main(int argc, char *argv[]) {

  ca::PC_XYZ pc;
  pc.push_back(ca::pcl_util::make_point_xyz<ca::P_XYZ>(0., 1., 2.));

  ca::NeaPoint pt;
  std::cerr << "sizeof(pt) = " << sizeof(pt) << std::endl;

  pt.x = 1;
  pt.y = 2;
  pt.z = 3;
  pt.x_origin = 4;
  pt.y_origin = 5;
  pt.z_origin = 6;
  pt.range_variance = 7;
  pt.x_variance = 8;
  pt.y_variance = 9;
  pt.z_variance = 10;
  pt.reflectance = 11;
  pt.time_sec = 12;
  pt.time_nsec = 13;
  pt.return_type = 14;
  pt.label = 15;

  pcl::PointCloud<ca::NeaPoint> neapc;
  neapc.push_back(pt);
  neapc.push_back(pt);

  pcl::PCLPointCloud2 pclpc;
  pcl::toPCLPointCloud2(neapc, pclpc);

  std::cerr << "pclpc.point_step = " << pclpc.point_step << std::endl;
  for ( size_t i=0; i < pclpc.fields.size(); ++i) {
    std::cerr << "pclpc.fields[" << i << "].name = " << pclpc.fields[i].name << std::endl;
    std::cerr << "pclpc.fields[" << i << "].datatype = " << pclpc.fields[i].datatype << std::endl;
    std::cerr << "pclpc.fields[" << i << "].count = " << pclpc.fields[i].count << std::endl;
    std::cerr << "pclpc.fields[" << i << "].offset = " << pclpc.fields[i].offset << std::endl;
    std::cerr << "\n";
  }

  ca::NeaPointf pt2;
  pt2.x = 1.;
  return 0;
}
