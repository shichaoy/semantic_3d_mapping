/*This files tests initializing a shared memory file with an
 * identifier and then accessing that file via two different proxy
 * gridmaps using the same identifer. Just checks basic parameters to
 * see if the same memory is being used by the proxygrid maps or not.
 * Also, it demonstrates that you only need to specify dimensions etc
 * during shared memory initialization*/

#include <ros/ros.h>
#include <gridmapping2/proxy_gridmap.hpp>
#include <gridmapping2/shared_gridmap.hpp>
int main(int argc, char *argv[]){
  ros::init(argc, argv, "shared_grid_map");
  ca::Vec3Ix grid_dimension(100,100,40);
  float grid_resolution(.5);
  std::string grid_identifier("test_map");
  ca::SharedGridMap gridmap(grid_identifier, grid_dimension, (float)grid_resolution);
  bool ok = gridmap.init();
  if (!ok) {
    ROS_ERROR_STREAM("shared gridmap failure");
    return -1;
  }

  ca::ProxyGridMap gridmap1(grid_identifier);
  ok = gridmap1.init();
  if (!ok) {
    ROS_ERROR_STREAM("proxy gridmap1 initialization failure");
    return -1;
  }

  ca::ProxyGridMap gridmap2(grid_identifier);
  ok = gridmap2.init();
  if (!ok) {
    ROS_ERROR_STREAM("proxy gridmap2 initialization failure");
    return -1;
  }
  if(gridmap1.dimension()==gridmap2.dimension()&&gridmap1.scrolloffset()==gridmap2.scrolloffset()){
    ROS_INFO_STREAM("members of scrollgrid class correctly read from shared memory"); 
    ROS_INFO_STREAM("dimensions and scroll_offset of grid1 "<<gridmap1.dimension().transpose()<<" -- "<<gridmap1.scrolloffset().transpose());
    ROS_INFO_STREAM("dimensions and scroll_offset of grid2 "<<gridmap2.dimension().transpose()<<" -- "<<gridmap2.scrolloffset().transpose());
  }else{
    ROS_ERROR_STREAM("memory was incorrectly shared");
    return -1;
  }

  return 0;
}
