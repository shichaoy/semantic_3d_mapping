
#include <ros/ros.h>

#include <gridmapping2/shared_gridmap.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "shared_grid_map");
  ca::Vec3Ix grid_dimension;
  double grid_resolution;
  std::string grid_identifier;
  ros::NodeHandle n("~");
  n.param<int>("/shared_gridmap/gridDimensionX", grid_dimension(0), 10);
  n.param<int>("/shared_gridmap/gridDimensionY", grid_dimension(1), 10);
  n.param<int>("/shared_gridmap/gridDimensionZ", grid_dimension(2), 5);
  n.param<std::string>("/shared_gridmap/gridIdentifier",grid_identifier,"shared_grid_map");
  n.param<double>("/shared_gridmap/gridResolution", grid_resolution, 0.5);
  ca::SharedGridMap gridmap(grid_identifier, grid_dimension, (float)grid_resolution);
  bool ok = gridmap.init();
  if (!ok) {
    ROS_ERROR_STREAM("shared gridmap failure");
    return -1;
  }

  ros::spin();

  return 0;
}
