#include <string>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<float, 11, 1> Vector_XXf;  // total label distribution, the same as grid sensor
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXf_row;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXi_row;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixX8umy;

class SuperPixel{
public:
	std::vector<int> pixel_indexes;
	
	int current_ind=0;  // superpixel index in all superpixels
	Vector_XXf label_distri;  // label distribution for the superpixel
	int label=-1; // label for this superpixel
	float max_label_prob=0;
public:
	int im_height=0;
	int im_width=0;

	SuperPixel(int h, int w)
	{
	      im_height = h;
	      im_width = w;	  
	      pixel_indexes.clear();
	}
	SuperPixel(int pixel_nums_in)
	{
	      pixel_indexes.clear();
	      pixel_indexes.resize(pixel_nums_in);
	}
	SuperPixel(int pixel_nums_in, int h, int w) {
	      pixel_indexes.clear();
	      pixel_indexes.resize(pixel_nums_in);
	      current_ind = 0;
	      im_height = h;
	      im_width = w;
	}
	~SuperPixel() {
	  if (pixel_indexes.size()>0)
	      pixel_indexes.clear();
	}
	
	void addIndexByRange(int row, int colStart, int colEnd);
	Eigen::Vector2f get_avg_pos();
	Eigen::Vector3f get_avg_color(const unsigned char * im);

};

Eigen::VectorXi get_color_from_label(int label);
Eigen::MatrixXi get_label_to_color_matrix();
int get_sky_label();

void read_superpixel_bin(const std::string superpixel_bin, std::vector<SuperPixel*>& all_superpixels,int img_width=1226,int img_height=370);
void read_superpixel_img(const std::string superpixel_img_file, std::vector<SuperPixel*>& all_superpixels);

void show_unary_result(const cv::Mat& unary_img, const cv::Mat& raw_rgb_img,cv::Mat& label_blended_img);
void show_pixel_crf_result(Eigen::VectorXi pix_labels, const cv::Mat& raw_rgb_img,cv::Mat& label_bw_img,cv::Mat& label_color_img, 
			   cv::Mat& label_blended_img);

void show_superpixel_crf_result(const std::vector<SuperPixel*>& all_superpixels, const Eigen::VectorXi & sp_labels, 
				const cv::Mat& raw_rgb_img,cv::Mat& label_blended_img);