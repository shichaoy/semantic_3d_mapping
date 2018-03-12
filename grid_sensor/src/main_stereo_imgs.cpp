#include <grid_sensor/grid_sensor.hpp>
#include "grid_sensor/grid_sensor_visualization.hpp"
#include <grid_sensor/util.hpp>
#include <fstream>
#include <iostream>
#include <string> 
#include <sstream>
#include <ctime>


using namespace std;


class dataset_wrapper
{
public:
      dataset_wrapper()
      {
	  n.param ("kitti/img_root_folder", img_root_folder, img_root_folder);	  
	  n.param ("kitti/raw_img_folder", raw_img_folder, raw_img_folder); 
	  n.param ("kitti/depth_img_folder", depth_img_folder, depth_img_folder);	  
	  n.param ("kitti/trajectory_file", trajectory_file, trajectory_file);
	  
	  n.param ("kitti/label_root_folder", label_root_folder, label_root_folder);
	  n.param ("kitti/label_bin_folder", label_bin_folder, label_bin_folder);
	  n.param ("kitti/superpixel_bin_folder", superpixel_bin_folder, superpixel_bin_folder);
	  n.param ("kitti/label_img_folder", label_img_folder, label_img_folder);
	  n.param ("kitti/evaluation_img_list", truth_img_list_file, truth_img_list_file);
	  n.param ("kitti/superpixel_img_folder", superpixel_img_folder, superpixel_img_folder);
	  n.param ("kitti/reproj_label_folder", saving_reproj_img_folder, saving_reproj_img_folder);
	  
	  n.param ("save_proj_imgs", save_proj_imgs, save_proj_imgs);
	  n.param ("use_crf_optimize", use_crf_optimize, use_crf_optimize);
    
	  raw_img_folder = img_root_folder + raw_img_folder;
	  depth_img_folder = img_root_folder + depth_img_folder;
	  trajectory_file = img_root_folder + trajectory_file;
	  label_bin_folder = label_root_folder + label_bin_folder;
	  superpixel_bin_folder = label_root_folder + superpixel_bin_folder;
	  label_img_folder = label_root_folder + label_img_folder;
	  truth_img_list_file = label_root_folder + truth_img_list_file;
	  superpixel_img_folder = label_root_folder + superpixel_img_folder;
	  saving_reproj_img_folder = label_root_folder + saving_reproj_img_folder;
	  
	  
	  grid_sensor = new ca::GridSensor(n);
	  grid_visualizer = new ca::GridVisualizer(nh);

	  img_couter=0;
	  total_img_ind=1000;
	  
	  image_width = 1226;
	  image_height = 370;
	  calibration_mat<<707.0912, 0, 601.8873, // kitti sequence 5
			  0, 707.0912, 183.1104,
			  0,   0,   1;

// 	  image_width = 1241;
// 	  image_height = 376;
// 	  calibration_mat<<718.856, 0, 607.1928, // kitti sequence 15
// 			   0, 718.856, 185.2157,
// 			   0,   0,   1;
	  
	  grid_sensor->set_up_calibration(calibration_mat,image_height,image_width);
	  
	  init_trans_to_ground<<1, 0, 0, 0,  
		    0, 0, 1, 0,
		    0,-1, 0, 1,
		    0, 0, 0, 1;
	  if (!read_all_pose(trajectory_file,total_img_ind+1,all_poses))    // poses of each frame
		ROS_ERROR_STREAM("cannot read file "<<trajectory_file);
	  
	  // set up label to color
	  frame_label_prob.resize(image_width*image_height,num_class);
	  grid_sensor->label_to_color_mat = get_label_to_color_matrix();
	  grid_sensor->sky_label = get_sky_label();
	  grid_visualizer->set_label_to_color(grid_sensor->label_to_color_mat,grid_sensor->sky_label);

	  // set up reprojection images and reprojection poses
	  if (save_proj_imgs){
		if (!read_evaluation_img_list(truth_img_list_file, evaluation_image_list))
		      ROS_ERROR_STREAM("cannot read file "<<truth_img_list_file);
		grid_sensor->set_up_reprojections(evaluation_image_list.rows());
    //     	     ROS_ERROR_STREAM("depth_img_name  "<<depth_img_name);
		Eigen::Matrix4f curr_transToWolrd;   // a camera space point multiplied by this, goes to world frame.
		curr_transToWolrd.setIdentity();
		for (int ind=0;ind<evaluation_image_list.rows();ind++)
		{
		    int img_couter=evaluation_image_list[ind];
		    VectorXf curr_posevec=all_poses.row(img_couter);
		    MatrixXf crf_label_eigen = Eigen::Map<MatrixXf_row>(curr_posevec.data(),3,4);
		    curr_transToWolrd.block(0,0,3,4) = crf_label_eigen;
		    curr_transToWolrd=init_trans_to_ground*curr_transToWolrd;
		    curr_transToWolrd(2,3)=1.0; // HACK set height to constant, otherwise bad for occupancy mapping.
		    grid_sensor->all_WorldToBodys[ind]=curr_transToWolrd.inverse();
		    grid_sensor->all_BodyToWorlds[ind]=curr_transToWolrd;
		}
		grid_sensor->reproj_frame_inds = evaluation_image_list;
	  }
	  if (crf_skip_frames>1)
	    std::cout<<"CRF opti skip frames:  "<<crf_skip_frames<<std::endl;
      }
      ros::NodeHandle n,nh;
      std::string img_root_folder,label_root_folder;      
      std::string raw_img_folder,depth_img_folder, label_bin_folder,trajectory_file,superpixel_bin_folder;
      std::string label_img_folder, truth_img_list_file,superpixel_img_folder;
      std::string saving_reproj_img_folder;
      bool save_proj_imgs,use_crf_optimize;
      
      ca::GridSensor* grid_sensor;
      ca::GridVisualizer* grid_visualizer;
    
      bool exceed_total=false;
      int crf_skip_frames=1;
      void process_frame()
      {
	    if (exceed_total)
		  return;
	    if (img_couter>total_img_ind)
	    {
	      ROS_ERROR_STREAM("Exceed maximum images");
	      exceed_total=true;
	      return;
	    }

	    char frame_index_c[256];
	    sprintf(frame_index_c,"%06d",img_couter);  // format into 6 digit
	    std::string frame_index(frame_index_c);

	    std::string img_left_name=raw_img_folder+frame_index+".png";
	    std::string depth_img_name=depth_img_folder+frame_index+".png"; 
	    std::string label_bin_name=label_bin_folder+frame_index+".bin";
	    std::string superpixel_bin_name=superpixel_bin_folder+frame_index+".bin";
	    std::string superpixel_img_name=superpixel_img_folder+frame_index+".png";
	    std::string label_img_name=label_img_folder+frame_index+"_color.png";
	    
	    cv::Mat raw_left_img = cv::imread(img_left_name, 1);    //rgb data
	    cv::Mat depth_img = cv::imread(depth_img_name, CV_LOAD_IMAGE_ANYDEPTH);      //CV_16UC1
	    cv::Mat label_img = cv::imread(label_img_name, 1);    //label rgb color image
	    cv::Mat superpixel_img = cv::imread(superpixel_img_name, 1);    //label rgb color image
	    if(raw_left_img.data)                             // Check for invalid input
		std::cout <<  "read image  "<<frame_index_c << std::endl ;	      
	    else{
		std::cout<<"cannot read left image  "<<img_left_name<<std::endl;
		return;
    	     }
	    if(!depth_img.data){                             // Check for invalid input		
		std::cout<<"cannot read depth image  "<<depth_img_name<<std::endl;
		return;
    	     }
	    Eigen::Matrix4f curr_transToWolrd;   // a camera space point multiplied by this, goes to world frame.
	    curr_transToWolrd.setIdentity();
	    VectorXf curr_posevec=all_poses.row(img_couter);
	    MatrixXf crf_label_eigen = Eigen::Map<MatrixXf_row>(curr_posevec.data(),3,4);
	    curr_transToWolrd.block(0,0,3,4) = crf_label_eigen;
	    curr_transToWolrd=init_trans_to_ground*curr_transToWolrd;
// 	    curr_transToWolrd(2,3)=1.0; // HACK set height to constant
	    
	    if (!read_label_prob_bin(label_bin_name,frame_label_prob))
	    {
		  ROS_ERROR_STREAM("cannot read label file "<<label_bin_name);
		  exceed_total=true;
		  if (use_crf_optimize)
		      return;
	    }
	    grid_sensor->AddDepthImg(raw_left_img, label_img, depth_img,superpixel_img,curr_transToWolrd,frame_label_prob);  // update grid's occupancy value and label probabitliy	    

	    if (img_couter%crf_skip_frames==0)  // test CRF every 4 frames
		if (use_crf_optimize)
		    grid_sensor->CRF_optimization(superpixel_bin_name);
	    
	    grid_visualizer->publishObstCloud(use_crf_optimize); //use_crf_optimize

	    if (save_proj_imgs) // for evaluation purpose
	    {
	      int img_in_vec = check_element_in_vector(img_couter,evaluation_image_list);
	      if (img_in_vec >= 0)
		  grid_sensor->actual_depth_imgs[img_in_vec]=depth_img.clone();  // set depth
	      if ( (img_in_vec>= 0) || (img_couter %4 ==0))  // re-project every four frames. projecting every frame takes much time.
	      {
		  ROS_INFO_STREAM("reproject images at  "<<img_couter);
		  grid_sensor->reproject_to_images(img_couter);  // project onto all past poses
	      }
	      for (int ind=0;ind<evaluation_image_list.rows();ind++)  // save all past images
	      {
		  int img_couter_cc = evaluation_image_list[ind];
		  if (img_couter_cc <= img_couter)
		  {
		      char frame_index_char[256];
		      sprintf(frame_index_char,"%06d",img_couter_cc);  // format into 6 digit
		      std::string frame_index_str(frame_index_char);
		      std::string reproj_label_img_bw_path=saving_reproj_img_folder+frame_index_str+"_bw.png";
		      std::string reproj_label_img_color_path=saving_reproj_img_folder+frame_index_str+"_color.png";
		      
		      cv::imwrite( reproj_label_img_color_path, grid_sensor->reproj_label_colors[ind]);
		      cv::imwrite( reproj_label_img_bw_path, grid_sensor->reproj_label_maps[ind]);
		  }
	      }
	    }
	    img_couter++;
      }
      
      int img_couter;
      int total_img_ind;
      int image_width;
      int image_height;

      // important. neveral change it manually, math the vector size.
      const int num_class=ca::Vector_Xxf().rows();      

      Eigen::MatrixXf all_poses;
      MatrixXf_row frame_label_prob;
      
      Eigen::Matrix4f init_trans_to_ground;  // initil transformation  // multiply a constant      
      
      Eigen::Matrix3f calibration_mat;
      
      // for evaluation
      VectorXi evaluation_image_list;
      std::vector<cv::Mat> reproject_label_imgs;
      std::vector<cv::Mat> reproject_depth_imgs; // depth buffer for images
      std::vector<cv::Mat> reproj_label_color;
};


int main(int argc, char *argv[]) {
      ros::init(argc, argv, "grid_sensor");
      ros::NodeHandle n;
      
      dataset_wrapper image_wrap;
      
      ros::Rate loop_rate(10);// hz      

      while (ros::ok())
      {
	    image_wrap.process_frame();
	    loop_rate.sleep(); 
      }
      
      ros::spin();
      return 0;
}