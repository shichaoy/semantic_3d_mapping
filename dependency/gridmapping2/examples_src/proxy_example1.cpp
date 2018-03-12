#include <ros/ros.h>

#include <gridmapping2/proxy_gridmap.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "proxy1");

  ca::ProxyGridMap gridmap;

//  gridmap.WaitUntilInit();
  bool ok = gridmap.init();
  if (!ok) {
    ROS_ERROR_STREAM("proxy gridmap failure");
    return -1;
  }

  ros::start();

  ros::Duration wait(4.0);
  wait.sleep();

  bool flip = true;

  ros::Rate rate(0.2);
  while (ros::ok()) {
/*      if (flip) {
        gridmap.set(1, 2, 3, 0);
      } else {
        gridmap.set(1, 2, 3, 28);
      }
      flip = !flip;
*/
      ROS_INFO("writer waiting for write lock");
      gridmap.WaitForWriteLock();

      ROS_INFO("writer starting to write");
//      gridmap.SetWriteLock(true);
      rate.sleep();
      ros::spinOnce();
      ROS_INFO("writer finished writing");
      gridmap.FreeWriteLock();
  }
  return 0;
}
