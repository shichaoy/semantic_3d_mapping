#ifndef SHARED_GRIDMAP_HPP_9FWOQEMW
#define SHARED_GRIDMAP_HPP_9FWOQEMW

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>

#include <scrollgrid/scrollgrid3.hpp>
#include <scrollgrid/dense_array3.hpp>

#include <gridmapping2/proxy_gridmap.hpp>

namespace ca
{

// TODO make this configurable

struct MapParameters {
  MapParameters() {
    shm_name = "shared_grid_map";
    dimension = ca::Vec3Ix(200,200,100);
    resolution = 0.1;
  }
  MapParameters(std::string name, ca::Vec3Ix dim, float res) {
    shm_name = name;
    dimension = dim;
    resolution = res;
  }
  std::string shm_name;
  ca::Vec3Ix dimension;
  float resolution;
};

class SharedGridMap {
public:
  SharedGridMap() : parameters_() { }
  SharedGridMap(std::string name, ca::Vec3Ix dim, float res):
    parameters_(name,dim,res) { }

  bool init() {
    namespace bipc = boost::interprocess;
    init_grid_semaphore_name_=parameters_.shm_name +"_init_grid_semaphore";
    init_semaphore_= new bipc::named_semaphore(bipc::open_or_create,init_grid_semaphore_name_.c_str(),0);
    size_t to_allocate = 0;
    to_allocate += sizeof(ScrollGrid3f);  // pointer
    to_allocate += sizeof(DenseArray3<uint8_t>);  // pointer
    to_allocate += sizeof(uint8_t)*(parameters_.dimension.prod()); // main storage part occupancy value
    to_allocate += sizeof(Eigen::Vector3f)*(parameters_.dimension.prod()); // by me, also RBG
    to_allocate += sizeof(Vector_Xxf)*(parameters_.dimension.prod()); // by me, label
    to_allocate += sizeof(int)*(parameters_.dimension.prod()); // by me, label
    to_allocate += 65535; // arbitrary buffer to account for my stupidity
    // nearest power of 2 -- desirable? necessary?
    //size_t to_allocate = (pow(2, ceil(log2(to_allocate))));
    to_allocate += (4096 - (to_allocate % 4096));
    ROS_INFO_STREAM("to_allocate = " << to_allocate);

    segment_ = new bipc::managed_shared_memory(bipc::create_only, parameters_.shm_name.c_str(), to_allocate);
    grid3_ = segment_->construct<ScrollGrid3f>("scrollgrid3f")(Eigen::Vector3f(0, 0, 0),
                                                             parameters_.dimension,
                                                             parameters_.resolution);  // pointer
    grid_data_ = segment_->construct<uint8_t>("grid_data")[grid3_->num_cells()]();  // main data occ value   uint8_t is the template type for dense_array3
    grid_data_wrapper_.reset(grid3_->dimension(), grid_data_); // get pointer

    rgb_data_ = segment_->construct<Eigen::Vector3f>("RGB_data")[grid3_->num_cells()]();  // by me, RBG data
    rgb_data_wrapper_.reset(grid3_->dimension(), rgb_data_); // main data by me, RBG data
    
    label_data_ = segment_->construct<Vector_Xxf>("Label_data")[grid3_->num_cells()]();  // by me, label data data
    label_data_wrapper_.reset(grid3_->dimension(), label_data_); // main data by me, label data

    grid_counts_ = segment_->construct<int>("Grid_count")[grid3_->num_cells()]();  // by me, label data data
    grid_counts_wrapper_.reset(grid3_->dimension(), grid_counts_); // main data by me, label data
    
    
    ROS_INFO("grid is created");
    init_semaphore_->post();
    return true;
  }

  virtual ~SharedGridMap() {
    namespace bipc = boost::interprocess;
    //segment.destroy<ScrollGrid3f>("scrollgrid3f");
    delete segment_;
    bipc::shared_memory_object::remove(parameters_.shm_name.c_str());
    init_semaphore_->remove(init_grid_semaphore_name_.c_str());
  }

private:
  MapParameters parameters_;
  boost::interprocess::managed_shared_memory* segment_;
  boost::interprocess::named_semaphore* init_semaphore_;
  std::string init_grid_semaphore_name_;
  ca::ScrollGrid3f* grid3_;
  ca::DenseArray3<uint8_t> grid_data_wrapper_;
  ca::DenseArray3<Eigen::Vector3f> rgb_data_wrapper_;
  ca::DenseArray3<Vector_Xxf> label_data_wrapper_;   //change accordingly in proxy_map
  ca::DenseArray3<int> grid_counts_wrapper_;   //change accordingly in proxy_map
  
  uint8_t* grid_data_;
  Eigen::Vector3f* rgb_data_;  // rgb
  Vector_Xxf* label_data_;  // float prob
  int* grid_counts_;

private:
  SharedGridMap(const SharedGridMap& other);
  SharedGridMap& operator=(const SharedGridMap& other);
};

} /* ca */

#endif /* end of include guard: SHARED_GRIDMAP_HPP_9FWOQEMW */
