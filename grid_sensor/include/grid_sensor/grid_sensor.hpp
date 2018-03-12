#ifndef GRID_SENSOR_HPP_I9SAOOSJ
#define GRID_SENSOR_HPP_I9SAOOSJ

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include <gridmapping2/proxy_gridmap.hpp>

#include <scrollgrid/scrollgrid3.hpp>
#include <scrollgrid/raycasting.hpp>
#include <scrollgrid/occ_raycasting.hpp>
#include <scrollgrid/scrolling_strategies.hpp>
#include <scrollgrid/grid_util.hpp>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>


#include "densecrf.h"


typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;


using namespace Eigen;
using namespace std;

namespace ca {

typedef Eigen::Matrix<mem_ix_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_men;


class GridSensor {
 public:

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

  //Parameters specific for rgbd grid  static const uint8_t CA_SG_UNKNOWN;
  uint8_t CA_SG_UNKNOWN;
  int32_t CA_SG_COMPLETELY_FREE;
  int32_t CA_SG_COMPLETELY_OCCUPIED;
  int32_t CA_SG_BARELY_FREE;
  int32_t CA_SG_BARELY_OCCUPIED;
  int32_t CA_SG_BELIEF_UPDATE_POS; // when hit
  int32_t CA_SG_BELIEF_UPDATE_NEG;

 public:

  GridSensor(ros::NodeHandle& n):
	initialized_(false),
	counter(0) {

	CA_SG_UNKNOWN = 128;  //128   255
	CA_SG_COMPLETELY_FREE = 0;
	CA_SG_COMPLETELY_OCCUPIED = 255;//255  254
	CA_SG_BARELY_FREE = 117;
	CA_SG_BARELY_OCCUPIED = 135; //129  128  //change grid_visualization.hpp accordingly
	CA_SG_BELIEF_UPDATE_POS = 10; // when hit  20
	CA_SG_BELIEF_UPDATE_NEG = 4;  // when pass through  2 (raw)
	  
	n.param<int>("Grid_SG_BARELY_FREE", CA_SG_BARELY_FREE, CA_SG_BARELY_FREE);
	n.param<int>("Grid_SG_BARELY_OCCUPIED", CA_SG_BARELY_OCCUPIED, CA_SG_BARELY_OCCUPIED);
	n.param<int>("Grid_SG_BELIEF_UPDATE_POS", CA_SG_BELIEF_UPDATE_POS, CA_SG_BELIEF_UPDATE_POS);
	n.param<int>("Grid_SG_BELIEF_UPDATE_NEG", CA_SG_BELIEF_UPDATE_NEG, CA_SG_BELIEF_UPDATE_NEG);
	
        n.param<std::string>("/grid_sensor/pointCloudFrame", point_cloud_frame_, "/camera_rgb_optical_frame");
        n.param<std::string>("/grid_sensor/worldFrame", world_frame_, "/world_frame");
        n.param<std::string>("/grid_sensor/sharedGridIdentifer",shared_grid_identifier_,"shared_grid_map");
	raw_cloud_pub = n.advertise<CloudXYZRGB> ("/rgbd_grid/raw_stereo_cloud", 50);
	raw_img_pub=n.advertise<sensor_msgs::Image>("/raw_img",10);
	raw_label_img_pub=n.advertise<sensor_msgs::Image>("/raw_label_img",10);
	raw_superpixel_img_pub = n.advertise<sensor_msgs::Image>("/superpixel_img",10);
	
	odom_pub=n.advertise<nav_msgs::Odometry>( "/odom_pose", 10 );


	proxy_=new ca::ProxyGridMap(shared_grid_identifier_);
        ROS_INFO_STREAM("Grid sensor :"<<point_cloud_frame_<<" , "<<world_frame_);
	
	n.param ("crf_iterations", crf_iterations, crf_iterations);
	n.param ("use_high_order", use_high_order, use_high_order);
	n.param ("use_high_order", use_hierarchical_inference, use_hierarchical_inference);
	
	n.param ("smooth_xy_stddev", smooth_xy_stddev, smooth_xy_stddev);
	n.param ("smooth_z_stddev", smooth_z_stddev, smooth_z_stddev);
	n.param ("smooth_weight", smooth_weight, smooth_weight);
	n.param ("appear_xy_stddev", appear_xy_stddev, appear_xy_stddev);
	n.param ("appear_z_stddev", appear_z_stddev, appear_z_stddev);
	n.param ("appear_rgb_stddev", appear_rgb_stddev, appear_rgb_stddev);
	n.param ("appear_weight", appear_weight, appear_weight);
	
	n.param ("depth_scaling", depth_scaling, depth_scaling);
	n.param ("depth_ignore_thres", depth_ignore_thres, depth_ignore_thres);
      }


  virtual ~GridSensor() {
   delete proxy_;
  }

 public:

  void preprocess_pose(ros::Time timnow,const cv::Mat& rgb_img, const cv::Mat& label_img,const cv::Mat& superpixel_img, 
		       const Eigen::Matrix4f& transToWorld);   
  void AddDepthImg(const cv::Mat& rgb_img,const cv::Mat& label_rgb_img,const cv::Mat& depth_img, const cv::Mat& superpixel_img, 
			  const Eigen::Matrix4f pose,MatrixXf_row& frame_label_prob); // pose is 12*1    possibly add label data here
  void reproject_to_images(int current_index);
  
  MatrixXf_men pixels_to_gridmem;  // store each pixel's correspondence to 3D occupancy grid memory index
  int crf_iterations=0;
  bool use_high_order=false;
  bool use_hierarchical_inference=false;
  void CRF_optimization(const string superpixel_bin_file);
  
  void ScrollGrid();
  void ClearGrid(const ca::Vec3Ix& start, const ca::Vec3Ix& finish);
  void Init();


  void UpdateProbability(int x,int y,int z, bool endpt){
    bool isadd=false,isremove=false;
    if(!grid3_->is_inside_grid(x,y,z))ROS_INFO_STREAM("updateprob:isinsideGrid"<<x<<" , "<<y<<" , "<<z);
    mem_ix_t mem_ix = grid3_->grid_to_mem(x, y, z);
    int32_t old_value = occ_array3_[mem_ix];
    if(!endpt){
      int32_t new_value = static_cast<int32_t>(occ_array3_[mem_ix])-CA_SG_BELIEF_UPDATE_NEG;
      new_value=std::max(CA_SG_COMPLETELY_FREE, new_value);
      occ_array3_[mem_ix] = static_cast<uint8_t>(new_value);
      isremove = (old_value>=CA_SG_BARELY_FREE && new_value<=CA_SG_BARELY_FREE); //REMOVE point from list
    }
    else{
      int32_t new_value = static_cast<int32_t>(occ_array3_[mem_ix])+CA_SG_BELIEF_UPDATE_POS;
      new_value=std::min(CA_SG_COMPLETELY_OCCUPIED, new_value);
      occ_array3_[mem_ix] = static_cast<uint8_t>(new_value);
      isadd = (old_value<CA_SG_BARELY_FREE && new_value>=CA_SG_BARELY_OCCUPIED);  //ADD point from list
    }
  } 

  Eigen::MatrixXi label_to_color_mat;
  int sky_label=1;
  
  // calibration related
  Eigen::Matrix3f calibration_mat;
  int im_height, im_width;
  void set_up_calibration(const Eigen::Matrix3f& calibration_mat_in,const int im_height_in,const int im_width_in);

  // for reprojection evaluation
  void set_up_reprojections(const int reprojection_frames);
  std::vector<cv::Mat> reproject_depth_imgs; // depth buffer for images
  std::vector<cv::Mat> actual_depth_imgs;
  std::vector<cv::Mat> reproj_label_colors;
  std::vector<cv::Mat> reproj_label_maps;
  std::vector<Eigen::Matrix4f> all_WorldToBodys; // all the frame poses need to project
  std::vector<Eigen::Matrix4f> all_BodyToWorlds; // all the frame poses need to project
  Eigen::VectorXi reproj_frame_inds;
  
 private:

  std::string point_cloud_frame_;
  std::string world_frame_;
  std::string shared_grid_identifier_;

  ca::ProxyGridMap* proxy_;  //occupancy grids
  ca::ScrollGrid3f* grid3_;
  ca::DenseArray3<uint8_t> occ_array3_;  // true data fields of occupancy value
  ca::DenseArray3<Vector3f> rgb_array3_;  // true data fields of rgb value
  ca::DenseArray3<Vector_Xxf> label_array3_;  // true data fields of label probability
  ca::DenseArray3<int> count_array3_;
  
  int counter;
  ca::ScrollGrid3f::Vec3 sensor_pos_;

  ros::Publisher raw_cloud_pub,raw_img_pub, raw_label_img_pub, raw_superpixel_img_pub,odom_pub;
  
  
  //Scrolling
  ca::ScrollForBaseFrame<float> scroll_strategy_;
  ca::Vec3Ix scroll_cells_;
  ca::Vec3Ix clear_i_min, clear_i_max, clear_j_min, clear_j_max, clear_k_min, clear_k_max;

  //Raycasting
  ca::ScrollGrid3f::Vec3 ray_end_pos_;
  ca::Vec3Ix sensor_pos_ijk_;
  ca::Vec3Ix ray_end_pos_ijk_;

  //Input point cloud is provided in local frame and therefore converted to point_cloud_global
  //If point cloud is global, assign it directly to point_cloud_global //TODO
  tf::TransformListener tf_listener_;
  PointCloudXYZ point_cloud_global_;
  tf::StampedTransform w_to_sensor_transform_;

  bool initialized_; 
    
  // depth to cloud   
  cv::Mat_<float> matx_to3d_, maty_to3d_;
  float depth_scaling=1000;
  float depth_ignore_thres=20.0;

  
  // CRF related
  float smooth_xy_stddev=3;
  float smooth_z_stddev=3;
  float smooth_weight=8;

  float appear_xy_stddev=160;
  float appear_z_stddev=40;
  float appear_rgb_stddev=4;
  float appear_weight=10;
	 
};

}
#endif
