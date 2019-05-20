# Semantic 3D mapping #
This package builds a semantic 3D grid map using stereo or RGBD input. It is suitable for large scale online mapping. The grid semantic label is optimized through hierarchical CRF.

**Authors:** [Shichao Yang](https://github.com/shichaoy), Yulan Huang

**Related Paper:**

* **Semantic 3D Occupancy Mapping through Efficient High Order CRFs**, IROS 2017, S. Yang, Y. Huang, S. Scherer  [**PDF**](https://arxiv.org/pdf/1707.07388.pdf)


# Installation
### Prerequisites
This code contains several ros packages. We have test in **ROS indigo/kinect**. Create or use existing a ros workspace.
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
if the download link breaks, please download [here](https://drive.google.com/open?id=1g0aR6qx8jCgkScPdA6nRt2BDWBRZ24Rb) and follow sh file to process it. Will fix it later.

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



# Notes #
1. Some mode parameters can be changed in ```grid_sensor/params/kitti_crf_3d.yaml ``` and ```grid_imgs.launch```

	if ```use_crf_optimize = false```, no 3D CRF optimization, 2D label is directly transferred to 3D.

	if ```use_crf_optimize = true, use_high_order = false ``` dense 3D CRF optimization runs.

	if ```use_crf_optimize = true, use_high_order = true ``` High order 3D CRF optimization runs. Superpixel data needs to be provided

2. This package only contains grid mapping, all the pre-processing steps are not included. See ```preprocess_data/README.md``` for details.
   
   Elas is used for computing dense disparity/depth. Dilation CNN is used for 2D semantic segmentation.
   
   SLIC is used for generating superpixel.  ORB SLAM is used to estimate camera pose.
   
3. If Grid sensor memory is not initialized properly, delete ```/dev/shm/sem.shared_grid_map...```

4. Our used ground truth image annotations are in ```preprocess_data/gt_label/``` Refer to paper experiments for more details.
