/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/



#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_util/point_cloud2_iterator.h>

void example1() {
  sensor_msgs::PointCloud2 pc2;
  pc2.height = 1;
  pc2.width = 4;
  ca::PointCloud2Modifier modifier(pc2);

  // setPointCloud2FieldsByString only works for "xyz" and "rgb"
  // setPointCloud2FieldsByString also adds appropriate padding
  // e.g. it assumes float "xyz" is 16 bytes, not 12
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  float point_data[] =   {1.0  , 2.0  , 3.0 ,
                          4.0  , 5.0  , 6.0 ,
                          7.0  , 8.0  , 9.0 ,
                          10.0 , 11.0 , 12.0};
  uint8_t color_data[] = {255  , 0    , 0  ,
                          0    , 255  , 0  ,
                          0    , 0    , 255,
                          255  , 255  , 255};
  size_t n_points = 4;

  // Define the iterators. When doing so, you define the Field you would like to
  // iterate upon and the type of you would like returned: it is not necessary
  // the type of the PointField as sometimes you pack data in another type (e.g.
  // 3 uchar + 1 uchar for RGB are packed in a float)

  ca::PointCloud2Iterator<float> iter_x(pc2, "x");
  ca::PointCloud2Iterator<float> iter_y(pc2, "y");
  ca::PointCloud2Iterator<float> iter_z(pc2, "z");

  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you
  // can create iterators for those: they will handle data packing for you (in
  // little endian RGB is packed as *,R,G,B in a float and RGBA as A,R,G,B)
  ca::PointCloud2Iterator<uint8_t> iter_r(pc2, "r");
  ca::PointCloud2Iterator<uint8_t> iter_g(pc2, "g");
  ca::PointCloud2Iterator<uint8_t> iter_b(pc2, "b");
  // Fill the PointCloud2
  for(size_t i=0; i<n_points;
      ++i, ++iter_x, ++iter_y, ++iter_z,
      ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = point_data[3*i+0];
    *iter_y = point_data[3*i+1];
    *iter_z = point_data[3*i+2];
    *iter_r = color_data[3*i+0];
    *iter_g = color_data[3*i+1];
    *iter_b = color_data[3*i+2];
  }

  pcl::io::savePCDFile("pc2iter_test1.pcd", pc2);
}

void example2() {

  sensor_msgs::PointCloud2 pc2;
  pc2.height = 1;
  pc2.width = 4;
  ca::PointCloud2Modifier modifier(pc2);

  std::cerr << "\n\nbefore set\n";
  int datatype = sensor_msgs::PointField::FLOAT32;
  modifier.setPointCloud2Fields(3,
                                "x", 1, datatype,
                                "y", 1, datatype,
                                "z", 1, datatype);
  std::cerr << "after set\n";
  std::cerr << "pc2.fields[0].offset = " << pc2.fields[0].offset << std::endl;
  std::cerr << "pc2.fields[1].offset = " << pc2.fields[1].offset << std::endl;
  std::cerr << "pc2.fields[2].offset = " << pc2.fields[2].offset << std::endl;

  float point_data[] =   {1.0  , 2.0  , 3.0 ,
                          4.0  , 5.0  , 6.0 ,
                          7.0  , 8.0  , 9.0 ,
                          10.0 , 11.0 , 12.0};
  size_t n_points = 4;
  ca::PointCloud2Iterator<float> iter_x(pc2, "x");
  ca::PointCloud2Iterator<float> iter_y(pc2, "y");
  ca::PointCloud2Iterator<float> iter_z(pc2, "z");
  // Fill the PointCloud2
  for(size_t i=0; i<n_points;
      ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = point_data[3*i+0];
    *iter_y = point_data[3*i+1];
    *iter_z = point_data[3*i+2];
  }

  pcl::io::savePCDFile("pc2iter_test2.pcd", pc2);

}

int main(int argc, char *argv[]) {

  std::cout << "example1\n";
  example1();
  std::cout << "example2\n";
  example2();

  return 0;
}
