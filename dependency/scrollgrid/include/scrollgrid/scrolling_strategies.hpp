#ifndef SCROLLING_STRATEGIES_HPP_ITK653BW
#define SCROLLING_STRATEGIES_HPP_ITK653BW

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <pcl_util/point_types.hpp>
#include <pcl_util/pcl_util.hpp>

#include "scrollgrid/scrollgrid3.hpp"

namespace ca
{

template<class Scalar>
class ScrollForBaseFrame {
public:
 typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

 typedef boost::shared_ptr<ScrollForBaseFrame> Ptr;

public:
  ScrollForBaseFrame() {
    ros::NodeHandle nh("~");

    // if you want to center box around sensor/robot, use (0,0,0)
    double x, y, z;
    nh.param<double>("/scrolling_strategies/target_sensor_to_center_x", x, 0);
    nh.param<double>("/scrolling_strategies/target_sensor_to_center_y", y, 0);
    nh.param<double>("/scrolling_strategies/target_sensor_to_center_z", z, 0);
    target_sensor_to_center_.x() = x;
    target_sensor_to_center_.y() = y;
    target_sensor_to_center_.z() = z;

    // how far do we get from "target" location before scrolling
    nh.param<double>("/scrolling_strategies/scroll_dist_thresh_x", x, 0);
    nh.param<double>("/scrolling_strategies/scroll_dist_thresh_y", y, 0);
    nh.param<double>("/scrolling_strategies/scroll_dist_thresh_z", z, 0);
    scroll_dist_thresh_.x() = x;
    scroll_dist_thresh_.y() = y;
    scroll_dist_thresh_.z() = z;
  }
  ScrollForBaseFrame(Vec3 target_sensor_to_center, Vec3 scroll_dist_thresh):
    target_sensor_to_center_(target_sensor_to_center),
    scroll_dist_thresh_(scroll_dist_thresh){
  }

  public:
  void setParameters(Vec3 target_sensor_to_center, Vec3 scroll_dist_thresh) {
    target_sensor_to_center_ = target_sensor_to_center;
    scroll_dist_thresh_ = scroll_dist_thresh;
  }

  ca::Vec3Ix compute(const tf::Transform& wv2laser,
                     const ca::ScrollGrid3<Scalar>& grid) {
    //Vec3 origin_laser;
    //origin_laser.x() = wv2laser.getOrigin().getX();
    //origin_laser.y() = wv2laser.getOrigin().getY();
    //origin_laser.z() = wv2laser.getOrigin().getZ();

    //tf::Quaternion q(wv2laser.getRotation());
    //// TODO scalar
    //Eigen::Quaternionf eq = ca::rot_cast< Eigen::Quaternionf >(q);
    //Eigen::AngleAxisf flip_around_z(.5*M_PI, Eigen::Vector3f(0, 0, 1) );
    //Eigen::AngleAxisf flip_around_x(.5*M_PI, Eigen::Vector3f(1, 0, 0) );

    // compute offset in laser frame, not world_view
    //Vec3 target_sensor_to_center_rot = eq*target_sensor_to_center_;

    // add desired baseframe offset
    //origin_laser += target_sensor_to_center_;
    //origin_laser += target_sensor_to_center_rot;

    tf::Vector3 target_sensor_to_center = wv2laser * ca::point_cast< tf::Vector3 >(target_sensor_to_center_);
    Vec3 origin_laser = ca::point_cast<Vec3>(target_sensor_to_center);  //sensor position in world frame.
    
//     ROS_ERROR_STREAM("sensor_pos = "<<origin_laser);

    // compute if it is too close to grid boundaries, and change scroll_cells
    // accordingly
    ca::Vec3Ix scroll_cells(0, 0, 0);
    // origin is in world_view (after cloud is preprocessed)
    // center is in world_view
    Vec3 center(grid.center());
    Vec3 radius(grid.radius());
    Vec3 deviation( (origin_laser-center) );
    Vec3 abs_deviation( deviation.cwiseAbs() );
    Scalar resolution = grid.resolution();
    scroll_cells[0] = static_cast<grid_ix_t>(round(deviation[0]/resolution));
    scroll_cells[1] = static_cast<grid_ix_t>(round(deviation[1]/resolution));
    scroll_cells[2] = static_cast<grid_ix_t>(round(deviation[2]/resolution));

    // how to interpret the scroll_dist_thresh parameter
    //
    // C = center
    // | = bounds
    // + = helicopter
    // <----- = deviation
    // |~~| = edge threshold
    //
    // --|---+-------C---------------|--
    //   |   <-------                |
    //   |~~|                        |

    // measures closeness to edge. so bigger is better
    Eigen::Matrix<bool, 3, 1> not_within_bounds(radius.array() < (abs_deviation+scroll_dist_thresh_).array());
    scroll_cells = scroll_cells.cwiseProduct(not_within_bounds.cast<grid_ix_t>());

    return scroll_cells;
  }

private:
  Vec3 target_sensor_to_center_;
  Vec3 scroll_dist_thresh_;

private:
  ScrollForBaseFrame(const ScrollForBaseFrame& other);
  ScrollForBaseFrame& operator=(const ScrollForBaseFrame& other);

};

template<class Scalar>
class ScrollDownForCoverage {
public:
 typedef Eigen::Matrix<Scalar, 3, 1> Vec3;

 typedef boost::shared_ptr<ScrollDownForCoverage> Ptr;

public:
  ScrollDownForCoverage() {
    ros::NodeHandle nh("~");

    double z;
    nh.param<double>("scrolling_strategies/scroll_cover_margin_z", z, 10);
    scroll_cover_margin_z_ = z;
  }

  virtual ~ScrollDownForCoverage() { }

public:

  ca::Vec3Ix compute(const ca::PC_XYZ& xyz,
                     const ca::ScrollGrid3<Scalar>& grid) {

    // do we need to scroll grid? find lowest relevant point in cloud.
    // we ignore points that fall outside xy boundaries of grid
    // TODO make this configurable
    ca::scrollgrid::Box<float, 2> box_xy(grid.min_pt().template head<2>().template cast<float>(),
                                         grid.max_pt().template head<2>().template cast<float>());
    int counted_pts = 0;
    float min_z = std::numeric_limits<float>::max();
    for (size_t i=0; i < xyz.size(); ++i) {
      const pcl::PointXYZ& p(xyz[i]);
      // TODO templatize on scalar
      Eigen::Vector2f pf2(p.getVector3fMap().head<2>());
      if (box_xy.contains(pf2)) {
        min_z = std::min(min_z, p.z);
        ++counted_pts;
      }
    }

    float current_min_z(grid.min_pt().z());

    ca::Vec3Ix scroll_cells(0, 0, 0);
    static const int MIN_POINTS_FOR_SCROLLING = 5;
    // scroll up, down or stay
    if (counted_pts >= MIN_POINTS_FOR_SCROLLING) {
      if (min_z < (current_min_z - scroll_cover_margin_z_)) {
        grid_ix_t num_cells = static_cast<int>(( (current_min_z-min_z)+scroll_cover_margin_z_)/grid.resolution() );
        scroll_cells = Vec3Ix(0, 0, -num_cells);
      } else if (min_z > (current_min_z + scroll_cover_margin_z_)) {
        grid_ix_t num_cells = static_cast<int>(( (min_z-current_min_z)-scroll_cover_margin_z_)/grid.resolution() );
        scroll_cells = Vec3Ix(0, 0, num_cells);
      } else {
        scroll_cells.setZero();
      }
    } else {
      scroll_cells.setZero();
    }
    ROS_INFO_STREAM("min_z " << min_z << ", current_min_z = " << current_min_z);

    return scroll_cells;
  }

private:
  Scalar scroll_cover_margin_z_;

private:
  ScrollDownForCoverage(const ScrollDownForCoverage& other);
  ScrollDownForCoverage& operator=(const ScrollDownForCoverage& other);

};

template<class Scalar>
class ScrollForBaseFrameAndDown {
public:
 typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
 typedef boost::shared_ptr<ScrollForBaseFrameAndDown> Ptr;

public:
  ScrollForBaseFrameAndDown() {

  }

  virtual ~ScrollForBaseFrameAndDown() { }

public:

  ca::Vec3Ix compute(const tf::Transform& wv2laser,
                     const ca::PC_XYZ& xyz,
                     const ca::ScrollGrid3<Scalar>& grid) {

    ca::Vec3Ix scroll_bf = bf_scroller_.compute(wv2laser, grid);
    ca::Vec3Ix scroll_zcov = zcov_scroller_.compute(xyz, grid);

    ca::Vec3Ix scroll_cells(scroll_bf);
    scroll_cells[2] = scroll_zcov[2];
    return scroll_cells;
  }

private:
  ScrollForBaseFrame<Scalar> bf_scroller_;
  ScrollDownForCoverage<Scalar> zcov_scroller_;

private:
  ScrollForBaseFrameAndDown(const ScrollForBaseFrameAndDown & other);
  ScrollForBaseFrameAndDown& operator=(const ScrollForBaseFrameAndDown & other);

};

} /* ca */

#endif /* end of include guard: SCROLLING_STRATEGIES_HPP_ITK653BW */
