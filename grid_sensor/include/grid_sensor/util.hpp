#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

// for recording timestamp
#include <iostream>
#include <fstream>
#include <string> 
#include <sstream>


int check_element_in_vector(const int element, const VectorXi &vec_check)
{
  for (int i=0;i<vec_check.rows();i++)
    if (element==vec_check(i))
	return i;
  return -1;
}


bool read_evaluation_img_list(const std::string truth_img_list, VectorXi& image_list)
{    
    if (std::ifstream(truth_img_list))
    {
	std::ifstream fPoses;
	fPoses.open(truth_img_list.c_str());
	int counter=0;
	std::vector<int> image_list_v;
	while(!fPoses.eof()){
	    std::string s;
	    std::getline(fPoses,s);
	    if(!s.empty()){
		stringstream ss;
		ss << s;
		int t;
		ss>>t;
		image_list_v.push_back(t);
		counter++;
	    }
	}
	fPoses.close();
	image_list.resize(counter);
	for (int i=0;i<counter;i++)
	{
	  image_list(i)=image_list_v[i];
	}
	return true;
    }
    else
      return false;
}


bool read_all_pose(const std::string trajectory_file, const int total_img_number,Eigen::MatrixXf& all_poses)
{
    if (std::ifstream(trajectory_file))
    {
      all_poses.resize(total_img_number,12);
      std::ifstream fPoses;    
      fPoses.open(trajectory_file.c_str());
      int counter=0;
      while(!fPoses.eof()){
	  std::string s;
	  std::getline(fPoses,s);
	  if(!s.empty()){
	      std::stringstream ss;
	      ss << s;
	      float t;
	      for (int i=0;i<12;i++){
		  ss >> t;  
		  all_poses(counter,i)=t;
	      }
	      counter++;
	      if (counter>=total_img_number)
		break;
	  }
      }
      fPoses.close();
      return true;
    }
    else
      return false;
}

bool read_label_prob(const std::string label_txt, MatrixXf_row& frame_label_prob)  // assumed mat size correct
{
    std::ifstream fLables;    
    fLables.open(label_txt.c_str());
    int counter=0;
    while(!fLables.eof()){
	std::string s;
	std::getline(fLables,s);
	if(!s.empty()){
	    std::stringstream ss;
	    ss << s;	  
	    float t;
	    for (int i=0;i<frame_label_prob.cols();i++){
		ss >> t;  
		frame_label_prob(counter,i)=t;
	    }
	    counter++;
	}
    }
    fLables.close();    
}


bool read_label_prob_bin(const std::string label_bin, MatrixXf_row& frame_label_prob)  // assumed mat size correct
{
    if (std::ifstream(label_bin))
    {  
	std::ifstream fLables(label_bin.c_str(),std::ios::in|ios::binary);
	if (fLables.is_open()){
	    int mat_byte_size=sizeof(float)*frame_label_prob.rows()*frame_label_prob.cols(); // byte number, make sure size is correct, or can use tellg to get the file size
	    float *mat_field=frame_label_prob.data();
	    
	    fLables.read((char*)mat_field,mat_byte_size);
	    fLables.close(); 
	}
	else{
	  ROS_ERROR_STREAM("Cannot open bianry label file "<<label_bin);
	  return false;
	}
	return true;
    }
    else
      return false;    
}

bool read_label_prob_mat(const std::string label_txt, cv::Mat& frame_label_mat, Eigen::MatrixXf& frame_label_prob)  // assumed mat size correct
{
    std::ifstream fLables;    
    fLables.open(label_txt.c_str());
    int counter=0;    
    while(!fLables.eof()){
	std::string s;
	std::getline(fLables,s);
	float t;
	if(!s.empty()){
	    std::stringstream ss;
	    ss << s;
	    for (int i=0;i<frame_label_mat.cols;i++){
		ss >> t; 
		frame_label_mat.at<float>(counter,i)=t;
	    }
	    counter++;
	}
    }        
    fLables.close();
    cv::Mat large_label_mat;    
    cv::resize(frame_label_mat,large_label_mat,cv::Size(),2,2);  //WRONG, resize is not easy 3D
    Eigen::Map<MatrixXf> frame_label_prob2(large_label_mat.ptr<float>(),large_label_mat.rows,large_label_mat.cols); 
    std::cout<<frame_label_prob2.cols()<<" "<<frame_label_prob2.rows()<<std::endl;
}


nav_msgs::Odometry mattoOdom(const Eigen::Matrix4f transToWorld){
    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x=transToWorld(0,3);
    odom_msg.pose.pose.position.y=transToWorld(1,3);
    odom_msg.pose.pose.position.z=transToWorld(2,3);
    Matrix3f pose_rot=transToWorld.block(0,0,3,3);
    Eigen::Quaternionf pose_quat(pose_rot);  
    
    odom_msg.pose.pose.orientation.w=pose_quat.w();
    odom_msg.pose.pose.orientation.x=pose_quat.x();
    odom_msg.pose.pose.orientation.y=pose_quat.y();
    odom_msg.pose.pose.orientation.z=pose_quat.z();
    return odom_msg;
}

Eigen::VectorXf homo_to_real_coord_vec(const Eigen::VectorXf &pt_homo_in){
    Eigen::VectorXf pt_out;
    if (pt_homo_in.rows()==4)
      pt_out=pt_homo_in.head(3)/pt_homo_in(3);
    else if (pt_homo_in.rows()==3)
      pt_out=pt_homo_in.head(2)/pt_homo_in(2);
    return pt_out;
}
Eigen::VectorXf real_to_homo_coord_vec(const Eigen::VectorXf &pt_in)
{
    Eigen::VectorXf pt_homo_out;
    int raw_rows=pt_in.rows();  
    
    pt_homo_out.resize(raw_rows+1);
    pt_homo_out<<pt_in,
		 1;
    return pt_homo_out;
}


/**
 * given a bounding box represented as two corners,
 * calculate the lines representing the edges of this box.
 * each line is represented as a pair of points in bb.
 */
template<class Scalar>
void BoundingBoxToLines(const Eigen::Matrix<Scalar, 3, 1>& min_pt,
                        const Eigen::Matrix<Scalar, 3, 1>& max_pt,
                        std::vector<geometry_msgs::Point>& bb) {

  Eigen::Matrix<Scalar, 3, 2> minmax;
  minmax.col(0) = min_pt;
  minmax.col(1) = max_pt;
  static const int seq[3*24] = {
    0,0,0,
    0,1,0,
    0,1,0,
    1,1,0,
    1,1,0,
    1,0,0,
    1,0,0,
    1,0,1,
    1,0,1,
    1,1,1,
    1,1,1,
    0,1,1,
    0,1,1,
    0,0,1,
    0,0,1,
    0,0,0,
    0,1,0,
    0,1,1,
    1,1,1,
    1,1,0,
    0,0,1,
    1,0,1,
    1,0,0,
    0,0,0
  };

  for (int row=0; row < 24; ++row) {
    int i = seq[3*row + 0];
    int j = seq[3*row + 1];
    int k = seq[3*row + 2];
    geometry_msgs::Point pm;
    pm.x = minmax(0, i);
    pm.y = minmax(1, j);
    pm.z = minmax(2, k);
    bb.push_back(pm);
  }

}

/**
 * make a wireframe box marker out of the box represented by min_pt, max_pt
 */
template<class Scalar>
void WireframeBoxMarker(const Eigen::Matrix<Scalar, 3, 1>& min_pt,
                        const Eigen::Matrix<Scalar, 3, 1>& max_pt,
                        visualization_msgs::Marker& bb_marker) {
  bb_marker.ns = "bb";
  bb_marker.id = 0;
  bb_marker.type = visualization_msgs::Marker::LINE_LIST;
  bb_marker.action = visualization_msgs::Marker::ADD;
  bb_marker.scale.x = 0.25;
  bb_marker.color.r = 0.5;
  bb_marker.color.g = 1.0;
  bb_marker.color.b = 0.2;
  bb_marker.color.a = 1.0;
  bb_marker.lifetime = ros::Duration();
  BoundingBoxToLines(min_pt, max_pt, bb_marker.points);
}



// Vectors should just be rotated as it is vector not a point, but for backwards compatibility the default does both rotation and translation
void transformVector3f(const tf::Transform &transform_c,
                       Eigen::Vector3f &vec, bool vector_just_does_rotation = false) {
  tf::Vector3 v;
  tf::Transform transform = transform_c;
  if(vector_just_does_rotation){
      tf::Vector3 zeros(0.0,0.0,0.0);
      transform.setOrigin(zeros);
  }
#if ROS_VERSION_MINIMUM(1, 9, 50) // groovy
  tf::vectorEigenToTF(vec.cast<double>(), v);
  Eigen::Vector3d vecd;
  tf::vectorTFToEigen(transform*v, vecd);
  vec = vecd.cast<float>();
#else
  tf::VectorEigenToTF(vec.cast<double>(), v);
  Eigen::Vector3d vecd;
  tf::VectorTFToEigen(transform*v, vecd);
  vec = vecd.cast<float>();
#endif
}



bool getCoordinateTransform(const tf::TransformListener &listener,
                            std::string fromFrame,
                            std::string toFrame,
                            double waitTime,
                            tf::StampedTransform &transform) {
  bool flag = true;
  double timeResolution = 0.001;
  unsigned int timeout = ceil(waitTime/timeResolution);
  unsigned int count = 0;

  while(flag) {
    flag = false;
    try {
      count++;
      listener.lookupTransform(toFrame,fromFrame,ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      flag = true;
      if(count>timeout) {
        flag = false;
        ROS_ERROR_STREAM("Cannot find transform from::"<<fromFrame << " to::"<< toFrame);
        return flag;
      }
      ros::Duration(timeResolution).sleep();
    }
  }
  return (!flag);
}
