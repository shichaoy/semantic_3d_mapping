#include <grid_sensor/grid_sensor.hpp>
#include <grid_sensor/util.hpp>
#include<boost/unordered_map.hpp>  
      
namespace ca {

typedef boost::unordered_map<mem_ix_t, int> map_gridTocrf;
typedef boost::unordered_map<int,mem_ix_t> map_crfTogrid;


template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

int pc_count=0;


//The callback function for clearing grid (clearCellsFun is defined in scrollgrid)
struct OccGridClear : public ca::ClearCellsFun {
  OccGridClear(GridSensor* rgbd) :
      rgbd_(rgbd) { }
  virtual void operator()(const Vec3Ix& start,
                          const Vec3Ix& finish) const {
    rgbd_->ClearGrid(start, finish);
  }
  GridSensor* rgbd_;
};


void GridSensor::preprocess_pose(ros::Time curr_time,const cv::Mat& rgb_img,const cv::Mat& label_img, const cv::Mat& superpixel_rgb_img,
				      const Eigen::Matrix4f& transToWorld){    
    // publish images
    cv_bridge::CvImage out_img_msg;
    out_img_msg.header.stamp=curr_time;
        
    cv::Mat rgb_img_small;
    cv::resize(rgb_img,rgb_img_small,cv::Size(), 0.5, 0.5);
    out_img_msg.image=rgb_img_small;
    out_img_msg.encoding=sensor_msgs::image_encodings::TYPE_8UC3; 
    raw_img_pub.publish(out_img_msg.toImageMsg());
            
    cv::Mat label_img_small;
    cv::resize(label_img,label_img_small,cv::Size(), 0.5, 0.5);
    out_img_msg.image=label_img_small;
    out_img_msg.encoding=sensor_msgs::image_encodings::TYPE_8UC3; 
    raw_label_img_pub.publish(out_img_msg.toImageMsg());  

    cv::Mat superpixel_rgb_img_small;
    cv::resize(superpixel_rgb_img,superpixel_rgb_img_small,cv::Size(), 0.5, 0.5);
    out_img_msg.image=superpixel_rgb_img_small;
    out_img_msg.encoding=sensor_msgs::image_encodings::TYPE_8UC3; 
    raw_superpixel_img_pub.publish(out_img_msg.toImageMsg());
    
    // publish pose odometry    
    nav_msgs::Odometry odom_msg=mattoOdom(transToWorld);
    odom_msg.header.frame_id = "/world";
    odom_msg.header.stamp=curr_time;
    odom_pub.publish(odom_msg);

    // create w_to_sensor_transform_, usef for scroll grid....	
    static tf::TransformBroadcaster br;
    sensor_pos_<<transToWorld(0,3),transToWorld(1,3),transToWorld(2,3);    
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(transToWorld(0,3),transToWorld(1,3),transToWorld(2,3)) );  
    transform.setBasis(tf::Matrix3x3(transToWorld(0,0),transToWorld(0,1),transToWorld(0,2),transToWorld(1,0),transToWorld(1,1),transToWorld(1,2),transToWorld(2,0),transToWorld(2,1),transToWorld(2,2)));
    w_to_sensor_transform_=tf::StampedTransform(transform, curr_time, "/world", "/base_frame");
    br.sendTransform(w_to_sensor_transform_);
}

void GridSensor::AddDepthImg(const cv::Mat& rgb_img,const cv::Mat& label_rgb_img, const cv::Mat& depth_img, const cv::Mat& superpixel_rgb_img,
				 const Eigen::Matrix4f transToWorld, MatrixXf_row& frame_label_prob) 
{
    if (!initialized_) {
      Init();
    }  
    
    if (!((frame_label_prob.array()>0).all()))
      ROS_ERROR_STREAM("detect negative element in initial probablity matrix");
    
    pc_count++;
    ros::Time curr_time=ros::Time::now();  
    assert(im_width == depth_img.cols);
    assert(im_height == depth_img.rows);    
    
    // publish odom, tf, images and so on.
    preprocess_pose(curr_time,rgb_img,label_rgb_img,superpixel_rgb_img,transToWorld);
    
    // just for visualization the cloud
    CloudXYZRGB::Ptr point_cloud(new CloudXYZRGB());
    point_cloud->header.frame_id = "/world";
    point_cloud->header.stamp = (curr_time.toNSec() / 1000ull);        
    
    ScrollGrid();
    
    boost::function<void (int,int,int,bool)> upProb( boost::bind( &GridSensor::UpdateProbability, this, _1,_2,_3,_4 ) );    
    proxy_->WaitForWriteLock();    
    if (grid3_->is_inside_box(sensor_pos_))
	sensor_pos_ijk_=grid3_->world_to_grid(sensor_pos_);

    float pix_depth;
    pcl::PointXYZRGB pt;        
    
    pixels_to_gridmem = MatrixXf_men::Ones(im_height,im_width)*(-1);  // according to test, different pixels may correspond to the same grids.
    int pixlabel;
    
    for (int32_t i=0; i<im_width*im_height; i++) {      // row by row
	int ux=i % im_width; int uy=i / im_width;        
	pix_depth=(float) depth_img.at<uint16_t>(uy,ux);
	pix_depth=pix_depth/depth_scaling; // NOTE scale match when saved depth img

	frame_label_prob.row(i).maxCoeff(&pixlabel);
	if (pixlabel == sky_label)  // NOTE don't project Sky label.
	  continue;
	if (pix_depth>0.1){
	      pt.z=pix_depth; pt.x=matx_to3d_(uy,ux)*pix_depth; pt.y=maty_to3d_(uy,ux)*pix_depth;
	      Eigen::VectorXf global_pt=homo_to_real_coord_vec(transToWorld*Eigen::Vector4f(pt.x,pt.y,pt.z,1));  // change to global position
	      pt.x=global_pt(0); pt.y=global_pt(1); pt.z=global_pt(2);
	      pt.r = rgb_img.at<cv::Vec3b>(uy,ux)[2]; pt.g = rgb_img.at<cv::Vec3b>(uy,ux)[1]; pt.b = rgb_img.at<cv::Vec3b>(uy,ux)[0];	      
// 	      point_cloud->points.push_back(pt);
	      
	      if (grid3_->is_inside_box(sensor_pos_)) {
		  if (pix_depth>depth_ignore_thres) // NOTE don't update far points
		    continue;
		  ray_end_pos_<<pt.x, pt.y, pt.z;
		  ray_end_pos_ijk_ = grid3_->world_to_grid(ray_end_pos_);
		  if (grid3_->is_inside_grid(ray_end_pos_ijk_)) {
		    ca::occupancy_trace(sensor_pos_ijk_,ray_end_pos_ijk_,upProb);       // update occupancy value
		    // update corresponding grid color
		    mem_ix_t gridmem = grid3_->grid_to_mem(ray_end_pos_ijk_);
		    
		    float old_count=count_array3_[gridmem];
		    rgb_array3_[gridmem] = ( (rgb_array3_[gridmem]*old_count) + Vector3f(pt.r,pt.g,pt.b) ) / (old_count+1);   // whether put in one big array
		    Vector_Xxf new_prob=frame_label_prob.row(i);
		    label_array3_[gridmem] = ( (label_array3_[gridmem]*old_count) + new_prob ) / (old_count+1); //or multiply as paper said
		    label_array3_[gridmem].normalize();   // (label_array3_[gridmem].sum())
		    count_array3_[gridmem] = old_count+1;

		  }
	      }
	}
    }
    float pix_depth_thre=0.5;
 // project each grid onto the image, to get grid-to-pixel correspondence.each grid correspond to at most one pixel
    Eigen::Matrix<float, 3, 1> grid_cell_w;
    for (int i=grid3_->first_i();i<grid3_->last_i();i++)
      for(int j=grid3_->first_j();j<grid3_->last_j();j++)
	for(int k=grid3_->first_k();k<grid3_->last_k();k++){
	  if (grid3_->is_inside_grid(ca::Vec3Ix(i,j,k))){
		uint8_t val = proxy_->get(i,j,k,grid_cell_w);
		if(val>CA_SG_BARELY_OCCUPIED){  // if an obstacles
		      mem_ix_t gridmem=grid3_->grid_to_mem(ca::Vec3Ix(i,j,k));
		      Vector3f local_pt=homo_to_real_coord_vec(transToWorld.inverse()*real_to_homo_coord_vec(grid_cell_w)); //curr_transToWolrd.inverse()
		      if (local_pt(2)<0)  // on the back, don't project
			continue;
		      if (local_pt(2)>40) // don't project on too far away points.
			continue;
		      Vector2f reproj_pixels=homo_to_real_coord_vec(calibration_mat*local_pt); // u v  (x y)
		      int x=int(reproj_pixels(0)); int y=int(reproj_pixels(1));
		      if ( x<im_width &  x>=0 & y<im_height &  y>=0){
			    pix_depth=(float) depth_img.at<uint16_t>(y,x);
			    pix_depth=pix_depth/depth_scaling; // NOTE scale match when saved depth img
			    if ( abs(local_pt(2)-pix_depth)<pix_depth_thre)   // multiple grid may map to same pixel....
				pixels_to_gridmem(y,x) = gridmem;
		      }
		}
	  }
	}

    proxy_->FreeWriteLock();
    
    point_cloud->header.frame_id="/world";
    point_cloud->header.stamp=(ros::Time::now().toNSec() / 1000ull);
    raw_cloud_pub.publish(*point_cloud);     // for visualization, we could use voxel filter 
}


void GridSensor::CRF_optimization(const string superpixel_bin_file)
{            
      int num_of_class=ca::Vector_Xxf().rows();
     // since I don't know the total variables in advance, instead of calling resizing all the time, I initially allocating a big matrix
      int crf_grid_nun_guess=grid3_->num_cells()/2;
      MatrixXf unary_mat(num_of_class, crf_grid_nun_guess);
      MatrixXf rgb_mat(3,   crf_grid_nun_guess);
      MatrixXf pose_mat(3,  crf_grid_nun_guess);
      
      int crf_grid_num_actual=0;  // number of grids to be optimized in CRF
      map_gridTocrf memInd_To_crfInd;  // store 3d grid memory index  mapped to CRF grid index.  one to one mapping      
      map_crfTogrid crfInd_To_memInd;  // store CRF grid index mapped to 3d grid memory index.      
      
      int negative_array_size=0;
      int positive_array_size=0;
      Eigen::Matrix<float, 3, 1> grid_cell_w;
      proxy_->WaitForWriteLock();
      for (int i=grid3_->first_i();i<grid3_->last_i();i++)   // loop over all grid space, if it is an obstacle, treat as a CRF node.
	for(int j=grid3_->first_j();j<grid3_->last_j();j++)
	  for(int k=grid3_->first_k();k<grid3_->last_k();k++){
	    if (grid3_->is_inside_grid(ca::Vec3Ix(i,j,k))){
		  uint8_t val = proxy_->get(i,j,k,grid_cell_w);
		  if(val>CA_SG_BARELY_OCCUPIED){  // if an obstacles, add to crf optimization.   
			mem_ix_t gridmem=grid3_->grid_to_mem(ca::Vec3Ix(i,j,k));
			Eigen::Vector3f rgbvalue=rgb_array3_[gridmem]; // also get world position.
			Vector_Xxf label_prob_value=label_array3_[gridmem];
			if (!((label_prob_value.array()>0).all())){
			     ROS_WARN_STREAM("Attention. There is non-positive probability in the array. Skip them.");
			     negative_array_size++;
			     if (negative_array_size<10)
			       std::cout<<"label_prob_value "<<label_prob_value.transpose()<<std::endl;
			     continue;
			}
			else
			      positive_array_size++;
			memInd_To_crfInd.insert(map_gridTocrf::value_type(gridmem,crf_grid_num_actual));
			crfInd_To_memInd.insert(map_crfTogrid::value_type(crf_grid_num_actual,gridmem));

			unary_mat.col(crf_grid_num_actual)=-(label_prob_value.array().log());  
			
			VectorXf grid_prob_unary=unary_mat.col(crf_grid_num_actual);
			if (!is_finite<VectorXf>(grid_prob_unary))
			     ROS_ERROR_STREAM("unary vector detect nan outputs"<<label_prob_value.transpose());
			
			rgb_mat.col(crf_grid_num_actual)=rgbvalue;
			pose_mat.col(crf_grid_num_actual)=grid_cell_w;
			
			crf_grid_num_actual++;
			if (crf_grid_num_actual>crf_grid_nun_guess-1){
			      crf_grid_nun_guess=crf_grid_nun_guess*2;
			      unary_mat.conservativeResize(num_of_class,crf_grid_nun_guess); // old values leaving untouched.
			      rgb_mat.conservativeResize(3,crf_grid_nun_guess);
			      pose_mat.conservativeResize(3,crf_grid_nun_guess);
			}
		  }
	    }
      }
      proxy_->FreeWriteLock();
      if (negative_array_size>0)
	    std::cout<<"negative  sign array is  "<<negative_array_size<<"  positive  "<<positive_array_size<<std::endl;
      
      // 3D grid CRF
      DenseCRF3D crf_grid_3d(crf_grid_num_actual, num_of_class);
      crf_grid_3d.setUnaryEnergy(unary_mat.leftCols(crf_grid_num_actual));
      crf_grid_3d.addPairwiseGaussian( smooth_xy_stddev, smooth_xy_stddev, smooth_z_stddev, pose_mat.leftCols(crf_grid_num_actual),
				       new PottsCompatibility( smooth_weight ) );
      crf_grid_3d.addPairwiseBilateral( appear_xy_stddev, appear_xy_stddev, appear_z_stddev, appear_rgb_stddev, appear_rgb_stddev, appear_rgb_stddev, 
				pose_mat.leftCols(crf_grid_num_actual),rgb_mat.leftCols(crf_grid_num_actual),new PottsCompatibility(appear_weight));
      MatrixXf crf_grid_output_prob;
      
      std::vector <SuperPixel*> image_2d_superpixels; // 2d image SuperPixel
      std::vector <SuperPixel*> grid_3d_superpixels;  // 3d grid "SuperPixels"
      crf_grid_3d.set_ho(use_high_order);
      if (use_high_order)
      {
	    read_superpixel_bin(superpixel_bin_file,image_2d_superpixels,im_width,im_height);
	    // construct 3D SuperPixels, find their indexs in the set of to be optimized grids.
	    
	    map_gridTocrf grid_mapto_sp; // each grid's belongings to superpixel
	    
	    for (int i=0;i<image_2d_superpixels.size();i++){
		  int sp_crf_grid_num=0;
		  vector<int> sp_crf_grid_inds;
		  for (int j=0;j<image_2d_superpixels[i]->pixel_indexes.size();j++){
			  int ux= image_2d_superpixels[i]->pixel_indexes[j] % im_width;  // pixel coordinate in one superpixel, horizontal
			  int uy= image_2d_superpixels[i]->pixel_indexes[j] / im_width;  // vertical
			  
			  if ( ((ux>im_width-1) || (ux<0)) || ((uy>im_height-1) || (uy<0)) ){
			    cout<<"pixel index out of image "<<image_2d_superpixels[i]->pixel_indexes[j]<<endl;
			    break;
			  }
			  //TODO may not be good... for example, if current pixel's depth is negative, cannot use?? multiple pixel map to same grid?
			  mem_ix_t grid_3d_ind = pixels_to_gridmem(uy,ux); 
			  if (grid_3d_ind<0)  // if not being set.
			      continue;
			  
			  map_gridTocrf::iterator iter=memInd_To_crfInd.find(grid_3d_ind);			  
			  if (iter != memInd_To_crfInd.end()){  // if found the key.
			      sp_crf_grid_num++;
			      sp_crf_grid_inds.push_back(iter->second);
			  }
		  }
		  if (sp_crf_grid_num>1){
		      SuperPixel* sp_3d = new SuperPixel(sp_crf_grid_num);
		      for (int k=0;k<sp_crf_grid_num;k++){
			  sp_3d->current_ind=k;
			  sp_3d->pixel_indexes[k]=sp_crf_grid_inds[k];  // 3d superpixel's indexes stores CRF node index
		      }
		      grid_3d_superpixels.push_back(sp_3d);
		  }
	    }

	    // get mean superpixel size
	    int total_sp_grid_num=0;
	    for (int i=0;i<grid_3d_superpixels.size();i++)
		total_sp_grid_num += grid_3d_superpixels[i]->pixel_indexes.size();
// 	    std::cout<<"crf_grid_num  "<< crf_grid_num_actual <<"  superpixel num   "<<grid_3d_superpixels.size()<<"  mean size  "
// 		      <<total_sp_grid_num/grid_3d_superpixels.size()<<" exp mean size  "<<crf_grid_num_actual/grid_3d_superpixels.size()<<std::endl;
// 	    cout<<"image_2d/3d_superpixels size "<<image_2d_superpixels.size()<<"  "<<grid_3d_superpixels.size()<<endl;

	    crf_grid_3d.hierarchical_high_order = use_hierarchical_inference;
	    if (use_hierarchical_inference)  
	    {	    
		  // get average feature of 3D superpixels.  //TODO 2d superpixel doesn't mean 3D superpixel.... should have some constraints.
		  MatrixXf unary_mat_sp(num_of_class, grid_3d_superpixels.size());
		  MatrixXf rgb_mat_sp(3,   grid_3d_superpixels.size());
		  MatrixXf pose_mat_sp(3,  grid_3d_superpixels.size());
		  for (int i=0;i<grid_3d_superpixels.size();i++)
		  {
			Vector_XXf unary_temp_sum=Vector_XXf::Zero();
			Vector3f pose_temp_sum=Vector3f::Zero();
			Vector3f rgb_temp_sum=Vector3f::Zero();
			int super_size=grid_3d_superpixels[i]->pixel_indexes.size();
			for (int px_ind: grid_3d_superpixels[i]->pixel_indexes )
			{
			      unary_temp_sum += unary_mat.col(px_ind);
			      pose_temp_sum += pose_mat.col(px_ind);
			      rgb_temp_sum += rgb_mat.col(px_ind);
			}
			unary_mat_sp.col(i)= unary_temp_sum / (float) super_size;
			pose_mat_sp.col(i)= pose_temp_sum / (float) super_size;
			rgb_mat_sp.col(i)= rgb_temp_sum / (float) super_size;
		  }
		  // reason superpixel's and grid's label together
		  // the 3 and 4 th pairwise energy is superpixel's.   smooth_xy_stddev   appear_xy_stddev
		  crf_grid_3d.setUnaryEnergy2(unary_mat_sp);
		  crf_grid_3d.addPairwiseGaussian( 0.1, 0.1, 0.1, pose_mat_sp,   //smooth_xy_stddev
						   new PottsCompatibility( smooth_weight ) );
		  crf_grid_3d.addPairwiseBilateral( 0.1, 0.1, 0.1, appear_rgb_stddev, appear_rgb_stddev, 
						    appear_rgb_stddev, pose_mat_sp,rgb_mat_sp,new PottsCompatibility( appear_weight ));

		  crf_grid_3d.all_3d_superpixels_=grid_3d_superpixels;

		  crf_grid_output_prob = crf_grid_3d.inference(crf_iterations);
	    }
	    else  // not my hierarchical, vineet's way?
	    {
		  crf_grid_3d.decoupled_high_order = false;
		  crf_grid_output_prob = crf_grid_3d.inference(crf_iterations);
	    }
      }
      else
      {
	    crf_grid_output_prob = crf_grid_3d.inference(crf_iterations);
      }

//    apply the grid CRF optimizationi results
      int grid_label_changed_num=0,old_max_label,new_max_label;
      for (int i=0;i<crf_grid_output_prob.cols();i++){
	    map_crfTogrid::iterator iter=crfInd_To_memInd.find(i);
	    if (iter != crfInd_To_memInd.end()){  // if found the key.
		label_array3_[iter->second].maxCoeff(&old_max_label);
		label_array3_[iter->second]=crf_grid_output_prob.col(i);
		for (int label_id=0;label_id<label_array3_[iter->second].rows();label_id++)
		{
		  if (label_array3_[iter->second](label_id) < 1e-8) // don't want prob to be to small.
		      label_array3_[iter->second](label_id) = 1e-8;
		}
		label_array3_[iter->second].normalize();
		label_array3_[iter->second].maxCoeff(&new_max_label);
		if (old_max_label!=new_max_label)
		    grid_label_changed_num++;
	    }
      }
//       std::cout<<"3d grid label changed number  "<<grid_label_changed_num<<"  all  "<<crf_grid_output_prob.cols()<<std::endl;
      
      if (use_high_order)
      {
	    for (int i=0;i<image_2d_superpixels.size();i++){
		delete image_2d_superpixels[i];
	    }
	    for (int i=0;i<grid_3d_superpixels.size();i++){
		delete grid_3d_superpixels[i];
	    }
	    image_2d_superpixels.clear();
	    grid_3d_superpixels.clear();
      }
}

void GridSensor::reproject_to_images(int current_index)
{
      proxy_->WaitForWriteLock();

      int max_label;
      Eigen::Matrix<float, 3, 1> grid_cell_w;
      // unproject pixel depth to grid to retrieve the label
      // TODO note that different pixel might retrieve the same 3D grid due to discretization
      float pix_depth;
      pcl::PointXYZRGB pt;
      for (int id=0;id<reproject_depth_imgs.size();id++)
      {
	  int proj_frame_id = reproj_frame_inds[id];
	  if (proj_frame_id <= current_index )  // only project to past frames
	  {
	    if (proj_frame_id > current_index -50)  // don't project on too earlier frames.
	    {
		for (int32_t i=0; i<im_width*im_height; i++)
		{      // row by row
		    int ux=i % im_width; int uy=i / im_width;
		    if ( ux>=im_width-1 ||  ux<=0 || uy>=im_height-1 ||  uy<=0)
		      continue;
		      
		    pix_depth=(float) actual_depth_imgs[id].at<uint16_t>(uy,ux);
		    pix_depth=pix_depth/depth_scaling; // NOTE scale match when saved depth img
		    if (pix_depth>0.1){
			  if (pix_depth>2*depth_ignore_thres) // NOTE don't update far points
			    continue;
			  pt.z=pix_depth; pt.x=matx_to3d_(uy,ux)*pix_depth; pt.y=maty_to3d_(uy,ux)*pix_depth;
			  Eigen::VectorXf global_pt=homo_to_real_coord_vec(all_BodyToWorlds[id]*Eigen::Vector4f(pt.x,pt.y,pt.z,1));  // change to global position
			  pt.x=global_pt(0); pt.y=global_pt(1); pt.z=global_pt(2);
			  
			  ray_end_pos_<<pt.x, pt.y, pt.z;
			  ray_end_pos_ijk_ = grid3_->world_to_grid(ray_end_pos_);
// 				cout<<pt.x<<"  "<<pt.y<<" "<<pt.z<<endl;
			  if (grid3_->is_inside_grid(ray_end_pos_ijk_)) {
				uint8_t val = proxy_->get(ray_end_pos_ijk_(0),ray_end_pos_ijk_(1),ray_end_pos_ijk_(2),grid_cell_w);
				if(val>CA_SG_BARELY_OCCUPIED){  // if an obstacles, add to crf optimization.   
				    mem_ix_t gridmem=grid3_->grid_to_mem(ray_end_pos_ijk_);
				    Vector_Xxf label_prob_value = label_array3_[gridmem];
				    label_prob_value.maxCoeff(&max_label);
				    reproj_label_maps[id].at<uint8_t>(uy,ux) = (uint8_t)max_label;
				    
				    reproj_label_colors[id].at<cv::Vec3b>(uy,ux)[0]=(uint8_t) label_to_color_mat(max_label,2);
				    reproj_label_colors[id].at<cv::Vec3b>(uy,ux)[1]=(uint8_t) label_to_color_mat(max_label,1);
				    reproj_label_colors[id].at<cv::Vec3b>(uy,ux)[2]=(uint8_t) label_to_color_mat(max_label,0);
				    // TODO could expansion, better for visualization
				}
			  }
		    }
		}
	    }
	  }
      }
      proxy_->FreeWriteLock();
}


void GridSensor::set_up_reprojections(const int reprojection_frames)
{
      reproject_depth_imgs.resize(reprojection_frames);
      reproj_label_colors.resize(reprojection_frames);
      reproj_label_maps.resize(reprojection_frames);
      all_WorldToBodys.resize(reprojection_frames);
      all_BodyToWorlds.resize(reprojection_frames);
      actual_depth_imgs.resize(reprojection_frames);
      for (int i=0;i<reproj_label_colors.size();i++){
	  reproject_depth_imgs[i]=cv::Mat(cv::Size(im_width,im_height),CV_32FC1,cv::Scalar(100));
	  reproj_label_colors[i]=cv::Mat(cv::Size(im_width,im_height),CV_8UC3,cv::Scalar(200,200,200));
	  reproj_label_maps[i]=cv::Mat(cv::Size(im_width,im_height),CV_8UC1,cv::Scalar(255));
      }
}

void GridSensor::set_up_calibration(const Eigen::Matrix3f& calibration_mat_in,const int im_height_in,const int im_width_in)
{
      calibration_mat = calibration_mat_in;
      im_height = im_height_in;
      im_width = im_width_in;
      matx_to3d_.create(im_height, im_width);
      maty_to3d_.create(im_height, im_width);
      float center_x=calibration_mat(0,2);  //cx
      float center_y=calibration_mat(1,2);  //cy
      float fx_inv=1.0/calibration_mat(0,0);  // 1/fx
      float fy_inv=1.0/calibration_mat(1,1);  // 1/fy
      for (int v = 0; v < im_height; v++) {
	for (int u = 0; u < im_width; u++) {
	  matx_to3d_(v,u) = (u - center_x) * fx_inv;
	  maty_to3d_(v,u) = (v - center_y) * fy_inv;
	}
      }
}


void GridSensor::ScrollGrid() {
      OccGridClear occ_grid_clear(this);
      proxy_->WaitForWriteLock();
      scroll_cells_ = scroll_strategy_.compute(w_to_sensor_transform_, *grid3_);     // moving steps in x y z   
      grid3_->scroll_and_clear(scroll_cells_, occ_grid_clear);  // will call cleargrid
      proxy_->FreeWriteLock();
}

void GridSensor::ClearGrid(const ca::Vec3Ix& start, const ca::Vec3Ix& finish) {
      for (int i=start(0);i< finish(0);i++) {
	for(int j=start(1);j< finish(1);j++) {
	  for(int k=start(2);k< finish(2);k++) {
	    proxy_->set(i, j, k, CA_SG_UNKNOWN);    //clear the past grids, then change them into unknown
	    proxy_->set_rgb(i,j,k,Vector3f::Zero());
	    proxy_->set_label(i,j,k,Vector_Xxf::Zero());
	    proxy_->set_count(i,j,k,0);
	  }
	}
      }
}


void GridSensor::Init() {
  proxy_->init();
  grid3_ = proxy_->GetScrollGrid();
  occ_array3_ = proxy_->GetStorage();
  rgb_array3_ = proxy_->GetRGBStorage();
  label_array3_ = proxy_->GetLabelStorage();    
  count_array3_ = proxy_->GetCountStorage();
  
  proxy_->WaitForWriteLock();
  ClearGrid(grid3_->scroll_offset(),grid3_->dimension()+grid3_->scroll_offset());
  proxy_->FreeWriteLock();
  
  initialized_=true;
  ROS_INFO("GridSensor: Grid Sensor initialized");
}

}
