#ifndef PROXY_GRIDMAP_HPP_JPCTAMDR
#define PROXY_GRIDMAP_HPP_JPCTAMDR

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/interprocess/managed_shared_memory.hpp>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_semaphore.hpp>

#include <scrollgrid/scrollgrid3.hpp>
#include <scrollgrid/dense_array3.hpp>

namespace ca
{

typedef Eigen::Matrix<float, 11, 1> Vector_Xxf;
  
class ProxyGridMap {

  public:
     ProxyGridMap():
      shared_memory_identifier_("shared_grid_map"),
      gridmap_initialized_(false),
      read_write_lock_(false) {
        /*namespace bipc=boost::interprocess;
	init_grid_semaphore_name_= "init_grid_semaphore";
	shared_grid_semaphore_name_ = "shared_grid_semaphore";
        init_semaphore_= new bipc::named_semaphore(bipc::open_or_create,init_grid_semaphore_name_.c_str(),0);*/
    }   
    ProxyGridMap(std::string grid_identifier):
      shared_memory_identifier_(grid_identifier),
      gridmap_initialized_(false),
      read_write_lock_(false) {}

    virtual ~ProxyGridMap() {
      delete segment_;
      semaphore_->remove(shared_grid_semaphore_name_.c_str());
      init_semaphore_->remove(init_grid_semaphore_name_.c_str());
    }

    bool init() {
      namespace bipc = boost::interprocess;
      init_grid_semaphore_name_= shared_memory_identifier_+"_init_grid_semaphore";
      shared_grid_semaphore_name_ = shared_memory_identifier_+"_shared_grid_semaphore";
      init_semaphore_= new bipc::named_semaphore(bipc::open_or_create,init_grid_semaphore_name_.c_str(),0);

      init_semaphore_->wait();
      ROS_INFO("grid creation is detected");  // get creating signal from shared_grid_map.hpp
      //TODO access control options
      segment_ = new bipc::managed_shared_memory(bipc::open_only, shared_memory_identifier_.c_str());
      {
        std::pair<ca::ScrollGrid3f*, size_t> result;
        result = segment_->find<ScrollGrid3f>("scrollgrid3f");  // get the pointer, same name as shared_gridmap.hpp
        // TODO error checking
        grid3_ = result.first;

      }
      {
        std::pair<uint8_t*, size_t> result;
        // TODO error checking
        result = segment_->find<uint8_t>("grid_data");
        grid_data_ = result.first;
      }
      {
        std::pair<Eigen::Vector3f*, size_t> result;
        // TODO error checking
        result = segment_->find<Eigen::Vector3f>("RGB_data");
        rgb_data_ = result.first;	
      }
      {
        std::pair<Vector_Xxf*, size_t> result;
        // TODO error checking
        result = segment_->find<Vector_Xxf >("Label_data");
        label_data_ = result.first;	
      }
      {
        std::pair<int*, size_t> result;
        // TODO error checking
        result = segment_->find<int >("Grid_count");
        grid_counts_ = result.first;	
      }

      
      //ROS_INFO_STREAM("grid3_->num_cells() = " << grid3_->num_cells());
      grid_data_wrapper_.reset(grid3_->dimension(), grid_data_);
      rgb_data_wrapper_.reset(grid3_->dimension(), rgb_data_);
      label_data_wrapper_.reset(grid3_->dimension(), label_data_);
      grid_counts_wrapper_.reset(grid3_->dimension(), grid_counts_);

      //Named semaphore initialization for synchronization
      semaphore_ = new bipc::named_semaphore(bipc::open_or_create, shared_grid_semaphore_name_.c_str(),1);

      ROS_INFO("grid inititalized");
      init_semaphore_->post();
      return true;
    }

    void FreeWriteLock() {
      semaphore_->post();
    }

    void WaitForWriteLock() {
      semaphore_->wait();
    }

    // TODO synchronization
    uint8_t get(grid_ix_t i, grid_ix_t j, grid_ix_t k, Eigen::Matrix<float, 3, 1>& grid_cell_in_world) {
      ca::Vec3Ix X(i,j,k);
      //WaitForWriteLock();     // TODO if want to test distancemap, just comment it.
      namespace bipc = boost::interprocess;
      uint8_t grid_cell;
      {
        if(!grid3_->is_inside_grid(X))ROS_ERROR_STREAM(X.transpose()<<" -- "<<grid3_->scroll_offset().transpose()<< " -- "<<grid3_->last_i()<<" , "<<grid3_->last_j()<<" , "<<grid3_->last_k()<<" , "<<i<<" , "<<j<< " , "<<k);
        grid_ix_t mem_ix = grid3_->grid_to_mem(X);
        grid_cell = grid_data_wrapper_[mem_ix];
        grid_cell_in_world = grid3_ ->grid_to_world(X);
      }
      //FreeWriteLock();
      return grid_cell;
    }

    void set(grid_ix_t i, grid_ix_t j, grid_ix_t k, uint8_t val) {
      namespace bipc = boost::interprocess;
      {
//        WaitForWriteLock();
        grid_ix_t mem_ix = grid3_->grid_to_mem(i, j, k);
        grid_data_wrapper_[mem_ix] = val;
//        FreeWriteLock();
      }
    }

//     // TODO could later put in get(), to get once
    Eigen::Vector3f get_rgb(grid_ix_t i, grid_ix_t j, grid_ix_t k, Eigen::Matrix<float, 3, 1>& grid_cell_in_world) {
      ca::Vec3Ix X(i,j,k);
      //WaitForWriteLock();     // TODO if want to test distancemap, just comment it.
      namespace bipc = boost::interprocess;
      Eigen::Vector3f grid_cell;
      {
        if(!grid3_->is_inside_grid(X))ROS_ERROR_STREAM(X.transpose()<<" -- "<<grid3_->scroll_offset().transpose()<< " -- "<<grid3_->last_i()<<" , "<<grid3_->last_j()<<" , "<<grid3_->last_k()<<" , "<<i<<" , "<<j<< " , "<<k);
        grid_ix_t mem_ix = grid3_->grid_to_mem(X);
        grid_cell = rgb_data_wrapper_[mem_ix];
        grid_cell_in_world = grid3_ ->grid_to_world(X);
      }
      //FreeWriteLock();
      return grid_cell;
    }

    void set_rgb(grid_ix_t i, grid_ix_t j, grid_ix_t k, Eigen::Vector3f val) {
      namespace bipc = boost::interprocess;
      {
//        WaitForWriteLock();
        grid_ix_t mem_ix = grid3_->grid_to_mem(i, j, k);
        rgb_data_wrapper_[mem_ix] = val;
//        FreeWriteLock();
      }
    }
  
    Vector_Xxf get_label(grid_ix_t i, grid_ix_t j, grid_ix_t k, Eigen::Matrix<float, 3, 1>& grid_cell_in_world) {
      ca::Vec3Ix X(i,j,k);
      //WaitForWriteLock();     // TODO if want to test distancemap, just comment it.
      namespace bipc = boost::interprocess;
      Vector_Xxf grid_cell;
      {
        if(!grid3_->is_inside_grid(X))ROS_ERROR_STREAM(X.transpose()<<" -- "<<grid3_->scroll_offset().transpose()<< " -- "<<grid3_->last_i()<<" , "<<grid3_->last_j()<<" , "<<grid3_->last_k()<<" , "<<i<<" , "<<j<< " , "<<k);
        grid_ix_t mem_ix = grid3_->grid_to_mem(X);
        grid_cell = label_data_wrapper_[mem_ix];
        grid_cell_in_world = grid3_ ->grid_to_world(X);
      }
      //FreeWriteLock();
      return grid_cell;
    }

    void set_label(grid_ix_t i, grid_ix_t j, grid_ix_t k, Vector_Xxf val) {
      namespace bipc = boost::interprocess;
      {
//        WaitForWriteLock();
        grid_ix_t mem_ix = grid3_->grid_to_mem(i, j, k);
        label_data_wrapper_[mem_ix] = val;
//        FreeWriteLock();
      }
    }
    void set_count(grid_ix_t i, grid_ix_t j, grid_ix_t k, int val) {
      namespace bipc = boost::interprocess;
      {
//        WaitForWriteLock();
        grid_ix_t mem_ix = grid3_->grid_to_mem(i, j, k);
        grid_counts_wrapper_[mem_ix] = val;
//        FreeWriteLock();
      }
    }  
  
    const ca::ScrollGrid3f* GetScrollGrid() const { return grid3_; }
    const ca::DenseArray3<uint8_t>& GetStorage() const { return grid_data_wrapper_; }
    const ca::DenseArray3<Eigen::Vector3f>& GetRGBStorage() const { return rgb_data_wrapper_; }    
    const ca::DenseArray3<Vector_Xxf>& GetLabelStorage() const { return label_data_wrapper_; }    
    const ca::DenseArray3<int>& GetCountStorage() const { return grid_counts_wrapper_; }    
    

    ca::ScrollGrid3f* GetScrollGrid() { return grid3_; }
    ca::DenseArray3<uint8_t>& GetStorage() { return grid_data_wrapper_; }
    ca::DenseArray3<Eigen::Vector3f>& GetRGBStorage() { return rgb_data_wrapper_; }
    ca::DenseArray3<Vector_Xxf>& GetLabelStorage() { return label_data_wrapper_; }     
    ca::DenseArray3<int>& GetCountStorage() { return grid_counts_wrapper_; }     

    //forwarded functions from scrollgrid (when we dont need the whole grid, just a little info
    const Vec3Ix& dimension() const { return grid3_->dimension(); }
    const Vec3Ix& scrolloffset() const {return grid3_->scroll_offset();}
    const ca::scrollgrid::Box<float,3>& box() const { return grid3_->box(); }

  private:
    boost::interprocess::managed_shared_memory* segment_;
    std::string shared_memory_identifier_,shared_grid_semaphore_name_,init_grid_semaphore_name_;
    ca::ScrollGrid3f* grid3_;
    ca::DenseArray3<uint8_t> grid_data_wrapper_;
    ca::DenseArray3<Eigen::Vector3f> rgb_data_wrapper_; // label/rgb data      //change based on shared_gridmap.
    ca::DenseArray3<Vector_Xxf> label_data_wrapper_; // label/rgb data
    ca::DenseArray3<int> grid_counts_wrapper_;   //change accordingly in proxy_map

    uint8_t* grid_data_;
    Eigen::Vector3f* rgb_data_;  // use float to later store probability of labels
    Vector_Xxf* label_data_;
    int* grid_counts_;

    bool lock;

  private:
    ProxyGridMap(const ProxyGridMap& other);
    ProxyGridMap& operator=(const ProxyGridMap& other);

    boost::interprocess::named_semaphore* semaphore_;
    bool read_write_lock_;

    //initialization conditions
    boost::interprocess::named_semaphore* init_semaphore_;
    bool gridmap_initialized_;
};

} /* ca */

#endif /* end of include guard: PROXY_GRIDMAP_HPP_JPCTAMDR */
