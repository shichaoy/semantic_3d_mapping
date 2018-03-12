# Semantic 3D mapping #
This package builds a semantic 3D grid map using stereo or RGBD input. It is suitable for large scale online mapping. The grid semantic label is optimized through hierarchical CRF.

**Authors:** [Shichao Yang](http://www.frc.ri.cmu.edu/~syang/), Yulan Huang

**Related Paper:**

* **Semantic 3D Occupancy Mapping through Efficient High Order CRFs **, IROS 2017, S. Yang, Y. Huang, S. Scherer  [**PDF**](http://www.frc.ri.cmu.edu/~syang/Publications/iros_2017.pdf)


# Installation
### Prerequisites
This code contains several ros packages. We test it in **ROS indigo/kinect + OpenCV 2.4**. Create or use existing a ros workspace.
```bash
mkdir -p ~/mapping_3d/src
cd ~/mapping_3d/src
catkin_init_workspace
git clone git@github.com:shichaoy/semantic_3d_mapping.git
```

### Download Data
```bash
cd semantic_3d_mapping/grid_sensor
sh download_data.sh
```
If ```wget``` not installed, ```sudo apt-get install wget ```

### Compile
```bash
cd ~/mapping_3d
catkin_make
```

# Running #
```bash
source devel/setup.bash
roslaunch grid_sensor grid_imgs.launch
```
You will see point cloud in Rviz. It also projects 3D grid onto 2D image for evaluation, stored at ```grid_sensor/dataset/crf_3d_reproj ```.

Some mode parameters can be changed under ```grid_sensor/params/ ```



# Notes #
1. This package only contains grid mapping, all the pre-processing steps are not included.
   
   Elas is used for computing dense disparity/depth. Dilation CNN is used for 2D semantic segmentation.
   
   SLIC is used for generating superpixel.  ORB SLAM is used to estimate camera pose.
   
2. If Grid sensor memory is not initialized properly, delete ```/dev/shm/sem.shared_grid_map...```

