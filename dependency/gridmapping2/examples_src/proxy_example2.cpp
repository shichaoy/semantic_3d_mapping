#include <ros/ros.h>

#include <gridmapping2/proxy_gridmap.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "proxy2");

  ca::ProxyGridMap gridmap;
//  gridmap.WaitUntilInit();
  bool ok = gridmap.init();
  if (!ok) {
    ROS_ERROR_STREAM("proxy gridmap failure");
    return -1;
  }

  ros::start();

  ros::Duration wait(2.0);
  wait.sleep();

  ros::Rate rate(2.0);

  Eigen::Matrix<float, 3, 1> x;
  while (ros::ok()) {
//      uint8_t val = gridmap.get(1, 2, 3, x);
//      ROS_INFO_STREAM("val = " << static_cast<int>(val));
    ROS_INFO("this is the reading process waiting");
    gridmap.WaitForWriteLock();
    ROS_INFO("this is the reading process started");

    rate.sleep();

    ROS_INFO("this is the reading process finished");
    gridmap.FreeWriteLock();
    ros::spinOnce();
  }

  return 0;
}
