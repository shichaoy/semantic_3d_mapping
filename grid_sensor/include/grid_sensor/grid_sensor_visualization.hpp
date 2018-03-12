#ifndef GRID_VISUALIZATIOM_HPP_IMNAOOSJ
#define GRID_VISUALIZATIOM_HPP_IMNAOOSJ

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <gridmapping2/proxy_gridmap.hpp>

#include <grid_sensor/util.hpp>

#include <scrollgrid/scrollgrid3.hpp>
#include <scrollgrid/grid_util.hpp>


namespace ca{

  typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixX8u_row;

  class GridVisualizer{
    typedef pcl::PointXYZRGB Point;
    typedef pcl::PointCloud<Point> PointCloud;
    public:
      GridVisualizer(ros::NodeHandle n):
        grid_obstacle_value_(135),   //TODO in accordance with grid_sensor
        initialized_(false),
        counter_(0){
        n.param<std::string>("/grid_visualize/worldFrame",world_frame_,"/world");
        n.param<std::string>("/grid_visualize/sharedGridIdentifer",shared_grid_identifier_,"shared_grid_map");
        boundary_marker.header.frame_id=world_frame_;
	obst_cloud_.reset(new PointCloud);

        obst_cloud_pub_=n.advertise<PointCloud>("/grid_visualize/obst_points", 100);
        boundary_pub_=n.advertise<visualization_msgs::Marker>("/grid_visualize/boundary_marker", 100);

	proxy_=new ca::ProxyGridMap(shared_grid_identifier_);
	got_pose=false;
	sky_label=1;// start from 0
	pc_counter=-1;
      }
    virtual ~GridVisualizer() {      
     delete proxy_;
    }
      /*Goes through all cells in grid and publishes obstacles,
        /TODO find a better way using changes or updates only*/
      
      void set_label_to_color(Eigen::MatrixXi& label_to_color_in, int sky_label_in)
      {
	  label_to_color=label_to_color_in.cast<uint8_t>();
	  sky_label = sky_label_in;
      }
      
      void get_Current_pose(const nav_msgs::OdometryConstPtr &msg)
      {
	got_pose=true;
	ca::Vec3Ix curr_grid_pos = grid3_->world_to_grid(ca::ScrollGrid3f::Vec3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
	current_layer=curr_grid_pos[2];
      }

      void publishObstCloud(const ros::TimerEvent&){
        publishObstCloud();
      }
      
      void publishObstCloud(bool use_label_color=false){
        if (!initialized_) {
          init();
        }
        obst_cloud_->clear();
        Eigen::Matrix<float, 3, 1> grid_cell_w;
        Point pt;
	  	
        proxy_->WaitForWriteLock();
        ROS_DEBUG("GridVisualizer:publishobstacles:got write lock");
	int layer=-2;
	if (got_pose)  layer=current_layer;
        //for (int i=0;i<grid_dimension_(0);i++)for(int j=0;j<grid_dimension_(1);j++)for(int k=0;k<grid_dimension_(2);k++){
	int max_label;
	for (int i=grid3_->first_i();i<grid3_->last_i();i++)
	  for(int j=grid3_->first_j();j<grid3_->last_j();j++)
	    for(int k=grid3_->first_k();k<grid3_->last_k();k++){
// 	    for(int k=-3;k<-2;k++){   //for 0.05 resolution k=9; for 0.075 resolution k=6  -6  -3
// 	    for (int k=layer;k<layer+1;k++){  // show only one layer
	      if (grid3_->is_inside_grid(ca::Vec3Ix(i,j,k))){
		    uint8_t val = proxy_->get(i,j,k,grid_cell_w);
		    if(val>grid_obstacle_value_){
      // 		      Vector_Xxf label_prob_value=proxy_->get_label(i,j,k,grid_cell_w); // also get world position.		      
			  label_array3_[grid3_->grid_to_mem(ca::Vec3Ix(i,j,k))].maxCoeff(&max_label);
			  if (max_label==sky_label)  // HACK don't show sky
			      continue;
			  Eigen::Vector3f rgbvalue=proxy_->get_rgb(i,j,k,grid_cell_w); // also get world position.
			  pt.x = grid_cell_w(0);
			  pt.y = grid_cell_w(1);
			  pt.z = grid_cell_w(2);
			  if (!use_label_color)
			  {
			      pt.r=(uint8_t)(rgbvalue(0));
			      pt.g=(uint8_t)(rgbvalue(1));
			      pt.b=(uint8_t)(rgbvalue(2));
			  }
			  else
			  {
			      pt.r=label_to_color(max_label,0);
			      pt.g=label_to_color(max_label,1);
			      pt.b=label_to_color(max_label,2);
			  }
			  obst_cloud_->points.push_back(pt);
      // 		      pt.intensity=255;
		    }
	      }
        }
        proxy_->FreeWriteLock();
        ROS_DEBUG("GridVisualizer:publishobstacles:write lock freed");
        pcl::toPCLPointCloud2(*obst_cloud_, cloud_);
        cloud_.header.stamp = ros::Time::now().toNSec()/1e3;
        cloud_.header.frame_id = world_frame_;
        obst_cloud_pub_.publish(cloud_);
        publishBoundary();
        ROS_DEBUG("GridVisualizer:publishing obstacle cloud - finish");
      }
      
      void publishBoundary(){
        WireframeBoxMarker(proxy_->box().min_pt(),proxy_->box().max_pt(),boundary_marker);
	
	boundary_marker.scale.x=0.1; 
	boundary_marker.scale.y=0.1;
	boundary_marker.scale.z=0.1;
        boundary_pub_.publish(boundary_marker);
        boundary_marker.points.clear();
      }

      /*proxy_->init() makes sure if shared gridmap has started or not*/
      void init(){
        proxy_->init();
        grid_dimension_= proxy_->dimension();
	grid3_ = proxy_->GetScrollGrid();
	label_array3_ = proxy_->GetLabelStorage();
        initialized_=true;
        ROS_INFO("GridVisualizer:RGBD Grid Visualization initialized");
      }
    private:
      std::string world_frame_;
      std::string shared_grid_identifier_;
      int publish_frequency_;
      uint8_t grid_obstacle_value_; // the value for cells which are obstacles in the grid

      ca::ProxyGridMap* proxy_;
      ca::ScrollGrid3f* grid3_;
      ca::DenseArray3<Vector_Xxf> label_array3_;  // true data fields  

      ca::Vec3Ix grid_dimension_;
      bool initialized_;
      int counter_;
      bool got_pose;
      int current_layer;
      //The output cloud
      PointCloud::Ptr obst_cloud_;
      ros::Publisher obst_cloud_pub_,boundary_pub_;
      visualization_msgs::Marker boundary_marker;
      //To transition to ros msg before publishing
      pcl::PCLPointCloud2 cloud_;
      
      // label to color
      MatrixX8u_row label_to_color;
      int sky_label;
      
      int pc_counter;
  };
}

#endif
