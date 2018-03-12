#include "densecrf.h"
#include "labelcompatibility.h"
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <string> 
#include <sstream>
#include <ctime>

#include <ros/package.h> // to find data path

using namespace std;

bool check_element_in_vector(const int element, const VectorXi &vec_check)
{
  for (int i=0;i<vec_check.rows();i++)
    if (element==vec_check(i))
	return true;
  return false;
}


template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

void read_truth_img_list(const std::string truth_img_list, VectorXi& image_list)
{
    ifstream fPoses;    
    fPoses.open(truth_img_list.c_str());
    int counter=0;
    vector<int> image_list_v;
    while(!fPoses.eof()){
	string s;
	getline(fPoses,s);
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
}

void read_label_prob_bin(const std::string label_bin, MatrixXf_row & frame_label_prob)  // assumed mat size correct
{
    std::ifstream fLables(label_bin.c_str(),std::ios::in|std::ios::binary);
    if (fLables.is_open()){
	int mat_byte_size=sizeof(float)*frame_label_prob.rows()*frame_label_prob.cols(); // byte number, make sure size is correct, or can use tellg to get the file size
	float *mat_field=frame_label_prob.data();
	
	fLables.read((char*)mat_field,mat_byte_size);	
	fLables.close(); 
    }
}

void get_unary(MatrixXf &unary_mat, int num_of_class, std::vector<SuperPixel*> all_superpixels, std::string label_bin) 
{
	MatrixXf_row frame_label_prob(1226*370, Vector_XXf().rows());
	read_label_prob_bin(label_bin, frame_label_prob);
	
	for (int j = 0; j < all_superpixels.size(); j ++)
	{
		SuperPixel* sp = all_superpixels[j];
		MatrixXf u_mat(num_of_class, sp->pixel_indexes.size());
		for(int i = 0; i < sp->pixel_indexes.size(); i ++) {
			u_mat.col(i) = frame_label_prob.row(sp->pixel_indexes[i]);
		}
		unary_mat.col(j) = -(u_mat.rowwise().mean().array().log());
	}
}


void comare_hier_dense_crf() {
	
	std::string data_root = ros::package::getPath("grid_sensor")+"/data_kitti/";
	std::string save_2d_path = ros::package::getPath("dense_crf")+"/test_data/";

	bool whether_save_img = true;
	bool visualize_img = true;
	int maxiter = 10;
	
	int img_couter = 10;  // frameid
	char frame_index_c[256];
	sprintf(frame_index_c,"%06d",img_couter);  // format into 6 digit
	std::string frame_index(frame_index_c);
	
	std::string superpixel_bin = data_root+"superpixel_bin/"+frame_index+".bin";
	std::string superpixel_rgb_path = data_root+"superpixel_img/"+frame_index+".png";
	std::string rgb_image_path = data_root+"rgb_img/"+frame_index+".png";		
	std::string unary_im_path = data_root+"label_visual/"+frame_index+"_color.png";
	std::string label_bin = data_root + "label_binary/"+frame_index+".bin";
	cout << "read image  "<< img_couter << endl;
	float smooth_x_stddev=3; float smooth_y_stddev=3; float smooth_weight=4;  // param same as dense 2d, next functions
	float appear_x_stddev=40; float appear_y_stddev=40;
	float appear_r_stddev=4; float appear_g_stddev=4; float appear_b_stddev=4; float appear_weight=8;
	
	// superpixel
	std::vector<SuperPixel*> all_superpixels;
	read_superpixel_bin(superpixel_bin, all_superpixels);

	cv::Mat raw_rgb_image = cv::imread(rgb_image_path, CV_LOAD_IMAGE_COLOR);
	cv::Mat superpixel_rgb = cv::imread(superpixel_rgb_path, CV_LOAD_IMAGE_COLOR);	
	
	int num_of_class = Vector_XXf().rows();
	int W = raw_rgb_image.cols, H = raw_rgb_image.rows;
	int num_pixels = W*H;

	DenseCRF3D* crf3d = new DenseCRF3D(num_pixels, num_of_class); // Dense CRF over superpixel.

	// unary
	MatrixXf_row frame_label_prob(W*H,num_of_class);
	read_label_prob_bin(label_bin,  frame_label_prob);
	frame_label_prob.transposeInPlace();
	MatrixXf_row pixel_unary = -(frame_label_prob.array().log());
	cout <<"frame_label_prob done  "<<pixel_unary.cols()<<"  "<<pixel_unary.rows()<<endl;

	// prepare feature list for pixels.
	MatrixXf posList(3, num_pixels);
	posList.setZero();
	MatrixXf rgb_img_array(3, num_pixels);
	rgb_img_array.setZero();	
	for (int x=0;x<W;x++)
	  for (int y=0;y<H;y++)
	  {
		int ind=y*W+x;
		posList(0,ind)=x;
		posList(1,ind)=y;
		
		rgb_img_array(0,ind)=raw_rgb_image.data[ind*3+0];
		rgb_img_array(1,ind)=raw_rgb_image.data[ind*3+1];
		rgb_img_array(1,ind)=raw_rgb_image.data[ind*3+2];
	  }

	crf3d->setUnaryEnergy( pixel_unary );	
	crf3d->addPairwiseGaussian( smooth_x_stddev, smooth_y_stddev, 1.0, posList, new PottsCompatibility( smooth_weight ) );  //x_stddev,y_stddev,weight
	crf3d->addPairwiseBilateral( appear_x_stddev, appear_y_stddev, 1, appear_r_stddev, appear_g_stddev, appear_b_stddev, 
							  posList, rgb_img_array, 
							  new PottsCompatibility( appear_weight ) ); //x_stddev,y_stddev,r_stddev, weight
	std::cout<<"finish set up first crf"<<std::endl;

	float sp_smooth_x_stddev=2;	float sp_smooth_y_stddev=2;	float sp_smooth_weight=4;
	float sp_appear_x_stddev=20;	float sp_appear_y_stddev=20;	
	float sp_appear_r_stddev=4;	float sp_appear_g_stddev=4;	float sp_appear_b_stddev=4;	float sp_appear_weight=8;
	
	crf3d->all_3d_superpixels_=all_superpixels;
	
	bool use_hierarchical_inference=true;
	
	crf3d->set_ho(true);
	crf3d->hierarchical_high_order = use_hierarchical_inference;
	if (use_hierarchical_inference)  
	{
	      // prepare posList
	      MatrixXf sp_posList(3, all_superpixels.size());
	      sp_posList.setZero();
	      for (int i = 0; i < all_superpixels.size(); i ++ )
	      {
		      SuperPixel* sp = all_superpixels[i];
		      sp_posList.col(i).head(2) = sp->get_avg_pos();
	      }
	      std::cout<<"all_superpixels size "<<all_superpixels.size()<<std::endl;
	      // prepare colorList
	      MatrixXf sp_rgb_img_array(3, all_superpixels.size());
	      sp_rgb_img_array.setZero();
	      for (int i = 0; i < all_superpixels.size(); i ++ )
	      {
		      SuperPixel* sp = all_superpixels[i];
		      sp_rgb_img_array.col(i) = sp->get_avg_color(raw_rgb_image.data);
	      }
		      
	      MatrixXf unary_mat_sp(num_of_class, all_superpixels.size());
	      for (int i=0;i<all_superpixels.size();i++)
	      {
		    Vector_XXf unary_temp_sum=Vector_XXf::Zero();
		    int super_size=all_superpixels[i]->pixel_indexes.size();
		    for (int j=0;j<super_size;j++)
		    {
			  unary_temp_sum += pixel_unary.col(all_superpixels[i]->pixel_indexes[j]);
		    }
		    unary_mat_sp.col(i)= unary_temp_sum / (float) super_size;
	      }
	      crf3d->setUnaryEnergy2(unary_mat_sp);
		
	      crf3d->addPairwiseGaussian( sp_smooth_x_stddev, sp_smooth_y_stddev, 1.0, sp_posList, new PottsCompatibility( sp_smooth_weight ) );  //x_stddev,y_stddev,weight
	      
	      crf3d->addPairwiseBilateral( sp_appear_x_stddev, sp_appear_y_stddev, 1, sp_appear_r_stddev, sp_appear_g_stddev, sp_appear_b_stddev, 
								sp_posList, sp_rgb_img_array, 
								new PottsCompatibility( sp_appear_weight ) ); //x_stddev,y_stddev,r_stddev, weight
	}
	std::cout<<"finish set up second crf"<<std::endl;

	VectorXi output_label_hier = crf3d->map(maxiter);
	std::cout << "Finish Hier CRF" << std::endl;

	
	
	DenseCRF2D crf(W, H, num_of_class);
	crf.setUnaryEnergy( pixel_unary );

	// Add simple pairwise potts terms
	crf.addPairwiseGaussian( smooth_x_stddev, smooth_y_stddev, new PottsCompatibility( smooth_weight ) );  //x_stddev,y_stddev,weight
// 	crf.addBilateral_pos(appear_x_stddev ,appear_y_stddev);
	// add a color dependent term (feature = xyrgb)
	crf.addPairwiseBilateral( appear_x_stddev, appear_y_stddev, appear_r_stddev, appear_g_stddev, appear_b_stddev, 
				  raw_rgb_image.data, new PottsCompatibility( appear_weight ) ); //x_stddev,y_stddev,r_stddev, weight

	VectorXi output_label_dense = crf.map(maxiter);
	std::cout << "Finish Dense CRF" << std::endl;

	
	
	if (visualize_img)
	    cv::imshow("Raw RGB image",raw_rgb_image );

	
	cv::Mat unary_im;
	unary_im = cv::imread(unary_im_path, CV_LOAD_IMAGE_COLOR);
	cv::Mat unary_label_blended_img;
	show_unary_result(unary_im, raw_rgb_image,unary_label_blended_img);
	if (visualize_img)
	    cv::imshow("Raw Unary result",unary_label_blended_img );

	
	{
	    cv::Mat label_bw_img, label_color_img, label_blended_img;
	    show_pixel_crf_result(output_label_hier, raw_rgb_image,label_bw_img,label_color_img,label_blended_img);

	    if (whether_save_img)
	    {
		std::string save_label_blend_path = save_2d_path+frame_index+"_hier_blend.png";
		cv::imwrite(save_label_blend_path, label_blended_img);
		std::string save_label_color_path = save_2d_path+frame_index+"_hier_color.png";
		cv::imwrite(save_label_color_path, label_color_img);
	    }

	    if (visualize_img)
	    {
		cv::imshow("Hierarchical CRF result",label_blended_img );
// 		cvWaitKey(0);
	    }
	}
	{
	    cv::Mat label_bw_img, label_color_img, label_blended_img;
	    show_pixel_crf_result(output_label_dense, raw_rgb_image,label_bw_img,label_color_img, label_blended_img);
	    if (whether_save_img)
	    {
		std::string save_label_blend_path = save_2d_path+frame_index+"_dense_blend.png";
		cv::imwrite(save_label_blend_path, label_blended_img);
		std::string save_label_color_path = save_2d_path+frame_index+"_dense_color.png";
		cv::imwrite(save_label_color_path, label_color_img);		
	    }
	    if (visualize_img)
	    {
		cv::imshow("2D Dense CRF result",label_blended_img );
		cvWaitKey(0);
	    }
	}
}


int main() {
	comare_hier_dense_crf();
}