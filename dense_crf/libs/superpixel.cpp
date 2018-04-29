#include "superpixel.h"
#include <fstream>
#include <iostream>
#include <string> 
#include <sstream>
#include <ctime>

void SuperPixel::addIndexByRange(int indRow, int colStart, int colEnd) {
	for (int k = colStart; k <= colEnd; k ++) {
		pixel_indexes[current_ind++] = indRow * im_width + k; // (im_height - indRow - 1) * im_width + k; // reverse save
	}
}

Eigen::Vector2f SuperPixel::get_avg_pos() {
	Eigen::Vector2f avg_pos;
	int sum_pos = 0;
	for (int pos : pixel_indexes) {
		sum_pos += pos;
	}
	
	int mean_pose=sum_pos / pixel_indexes.size();

	avg_pos.resize(2);
	avg_pos[0] = mean_pose % im_width;  // image x
	avg_pos[1] = mean_pose / im_width;  // image y
	return avg_pos;
}

Eigen::Vector3f SuperPixel::get_avg_color(const unsigned char * im) {
	Eigen::Vector3f avg_color;
	avg_color.resize(3);
	for (int pos : pixel_indexes)
	{
		avg_color[0] += im[pos*3];
		avg_color[1] += im[pos*3+1];
		avg_color[2] += im[pos*3+2];
	}
	avg_color /= pixel_indexes.size();
	return avg_color;
}

int16_t* get_frame_superpixels(int row_num, int col_num) {
	static int16_t* frame_superpixels = nullptr; 
	if (frame_superpixels == nullptr){
		frame_superpixels = new int16_t[row_num*col_num];
	}
	return frame_superpixels;
}

int get_sky_label()
{
	return 1;  // starts from 0.
}
Eigen::MatrixXi get_label_to_color_matrix()
{
  	Eigen::MatrixXi label_to_color(Vector_XXf().rows(),3);
	label_to_color<<128,	0,	0,    // 1. building  // our 11 class  //TODO this conversion should be the same with all other places
			128,	128,	128,  // 2. sky
			128,	64,	128,  // 3. road
			128,	128,	0,    // 4. vegetation
			0,	0,	192,  // 5. sidewalk
			64,	0,	128,  // 6. car
			64,	64,	0,    // 7. pedestrian
			0,	128,	192,  // 8 cyclist
			192,	128,	128,  // 9. signate
			64,	64,	128,  // 10. fence
			192,	192,	128;  // 11. pole
	return label_to_color;
}


Eigen::VectorXi get_color_from_label(int label) {
	return get_label_to_color_matrix().row(label);
}

void show_unary_result(const cv::Mat& unary_img, const cv::Mat& raw_rgb_img,cv::Mat& label_blended_img) {
	addWeighted(raw_rgb_img, 0.5, unary_img, 0.5, 0, label_blended_img);
}

void show_pixel_crf_result(Eigen::VectorXi pix_labels, const cv::Mat& raw_rgb_img,cv::Mat& label_bw_img,cv::Mat& label_color_img, 
			   cv::Mat& label_blended_img) {
	int im_width = raw_rgb_img.cols;
	int im_height = raw_rgb_img.rows;
	Eigen::MatrixXi mat = Eigen::Map<MatrixXi_row>(pix_labels.data(), im_height, im_width);
	
	MatrixX8umy mat8u = mat.cast<uint8_t>();
	
	label_bw_img=cv::Mat(mat.rows(), mat.cols(), CV_8UC1, mat8u.data());
	
	label_color_img=cv::Mat(cv::Size(im_width, im_height), CV_8UC3, cv::Scalar(0));
	for (int i=0; i<im_height; i++) {
		for(int j=0; j<im_width; j++) {
			int label = (int) label_bw_img.at<uint8_t>(i, j);
			Eigen::VectorXi color = get_color_from_label(label);

			label_color_img.at<cv::Vec3b>(i, j)[0] = (uint8_t) color(2);//b
			label_color_img.at<cv::Vec3b>(i, j)[1] = (uint8_t) color(1);//g
			label_color_img.at<cv::Vec3b>(i, j)[2] = (uint8_t) color(0);//r
		}
	}
	addWeighted(raw_rgb_img, 0.5, label_color_img, 0.5, 0, label_blended_img);
// 	imshow("Display Window result",label_color_img );
}


void show_superpixel_crf_result(const std::vector<SuperPixel*>& all_superpixels, const Eigen::VectorXi &sp_labels, 
				const cv::Mat& raw_rgb_img, cv::Mat& label_blended_img) {
	// assume input is >= 1 all_superpixels
	int im_width = all_superpixels[0]->im_width;
	int im_height = all_superpixels[0]->im_height;
	
 	cv::Mat label_color_img(cv::Size(im_width, im_height), CV_8UC3, cv::Scalar(0));
	for (int s=0 ; s< all_superpixels.size(); s++) {
		SuperPixel* sp = all_superpixels[s];
		int label = sp_labels(s);
		for(int i: sp->pixel_indexes) {
			int vertical_x = i/im_width;
			int horizontal_y = i%im_width;
			Eigen::VectorXi color = get_color_from_label(label);

			label_color_img.at<cv::Vec3b>(vertical_x, horizontal_y)[0] = color(2);//b
			label_color_img.at<cv::Vec3b>(vertical_x, horizontal_y)[1] = color(1);//g
			label_color_img.at<cv::Vec3b>(vertical_x, horizontal_y)[2] = color(0);//r
		}
	}
	addWeighted(raw_rgb_img, 0.5, label_color_img, 0.5, 0, label_blended_img);
// 	imshow("Display Window Superpixel CRF",label_blended_img);
}


void read_superpixel_img(const std::string superpixel_img_file, std::vector<SuperPixel*>& all_superpixels)
{  	// this method is slow compared to bin, even exclude imread
       cv::Mat sp_img = cv::imread(superpixel_img_file,CV_LOAD_IMAGE_ANYDEPTH);

       int max_sp_num=170; // TODO match  row_num in  read_superpixel_bin
       int im_width = sp_img.cols; int im_height = sp_img.rows;
	
       std::vector<int> sp_num_vec(max_sp_num);
       std::vector<SuperPixel*> all_superpixels_raw(max_sp_num);
       for (int i=0;i<max_sp_num;i++)
       {
	    all_superpixels_raw[i]=new SuperPixel(im_height,im_width);
       }
       for (int y=0;y<im_height;y++)
	 for (int x=0;x<im_width;x++)
	 {
	    all_superpixels_raw[sp_img.at<uint16_t>(y,x)]->pixel_indexes.push_back(y*im_width+x);
	 }
	for (int i=0;i<max_sp_num;i++)
	{
	  if (all_superpixels_raw[i]->pixel_indexes.size()>0)
	      all_superpixels.push_back(all_superpixels_raw[i]);
	}
	std::cout<<"total valid sp "<<all_superpixels.size()<<std::endl;
}

void read_superpixel_bin(const std::string superpixel_bin, std::vector<SuperPixel*>& all_superpixels,int im_width,int im_height) {
	// TODO move to config
	int max_sp_num = 170; int col_num = 2077; //72   // match max_sp_num in read_superpixel_img
	
	int16_t* frame_superpixels = get_frame_superpixels(max_sp_num, col_num);//new int16_t[max_sp_num*col_num];
	std::ifstream fLables(superpixel_bin.c_str(),std::ios::in|std::ios::binary);

	bool visualize_segment=false;
	cv::Mat clique_segment_img;
	if (visualize_segment)
	      clique_segment_img = cv::Mat(cv::Size(im_width, im_height), CV_8UC1, cv::Scalar(0));
	
	if (fLables.is_open()){
		int mat_byte_size=sizeof(int16_t)*max_sp_num*col_num;
		fLables.read((char*)frame_superpixels,mat_byte_size);
		fLables.close(); 
		
		// find number of superpixel in this frame
		int segmentCount = 0;
		for (int i=0; i<max_sp_num; i++) {
			if (frame_superpixels[i*col_num] > 0) {
				segmentCount ++;
			} else {
				break;
			}
		}
		all_superpixels.resize(segmentCount);
		
		// find number of pixels per superpixel
		for (int i=0; i<segmentCount; i++) {
			int pixel_block_num = frame_superpixels[i*col_num] /3;
// 			std::cout << i << " pixel_block_num " <<pixel_block_num << std::endl;
			int pixel_num = 0;
			for (int j=0; j<pixel_block_num; j++) {
				int ind = i*col_num+3*j+1;
				int indStart = frame_superpixels[ind+1];
				int indEnd = frame_superpixels[ind+2];
				
				pixel_num += indEnd - indStart + 1;
			}
			all_superpixels[i] = new SuperPixel(pixel_num, im_height, im_width);
		}
		
		// set indexes for each superpixel
		for (int i=0; i<segmentCount; i++) {
			int pixel_block_num = frame_superpixels[i*col_num] /3;
			int curInd = 0;
			for (int j=0; j<pixel_block_num; j++) {
				int ind = i*col_num+3*j+1;
				int indRow = frame_superpixels[ind];
				int indStart = frame_superpixels[ind+1];
				int indEnd = frame_superpixels[ind+2];
				all_superpixels[i]->addIndexByRange(indRow, indStart, indEnd);
				
				if (visualize_segment)
				    for (int ii=indStart;ii<=indEnd;ii++)
					clique_segment_img.at<uchar>(indRow,ii)=i*3;
			}
		}
		if (visualize_segment)
		  	cv::imshow("Superpixel segmentation",clique_segment_img);
	} //else {
// 		ROS_ERROR_STREAM("Cannot open bianry label file "<<superpixel_bin);
// 	}
}

