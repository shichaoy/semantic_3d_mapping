We explain preprocessing steps more details here.

   Elas is used for computing dense disparity/depth. Dilation CNN is used for 2D semantic segmentation.
   
   SLIC is used for generating superpixel.  ORB SLAM is used to estimate camera pose.

### Disparity/Depth
We use [Elas](http://www.cvlibs.net/software/libelas/) to compute dense disparity and depth. Ros ELas is preferred due to easy interface. Save the depth image (mm unit) in CV_16UC1 format.


### Superpixel
We use [SLIC](http://www.cvlibs.net/projects/displets/), external/stereoslic. A complete copy with modification is in this repo /preprocess_data/code/superpixel.
```bash
cmake .
make
```
then ``` ./stereoslic aa.png -s 150 ``` It will read sample images in raw_imgs/, then save to superpixel_ind/. To efficiently find pixel belonging to which superpixel in the latter c++ code, run 'superpixel_img2bin.m' to change superpixel image into binary data. The script also provides visualization of binary data. The general idea is to save a 170*2077 matrix, where 170 is maximum number of superpixel, each row represents one superpixel and its containing pixel location in this format:  [ num of triples*3, [row1, start_col1, end_col1],[row2, start_col2, end_col2]... ]


### CNN 2D
We use [Dilation Network](https://github.com/fyu/dilation) to predict the 2D semantic segmentation, which can be replaced by other segmentation. Our modified prediction code is under /preprocess_data/code/cnn_2d/predict_dilanet.py  Change caffe path at the top and dataset path in batch_predict_kitti().  It will save the class probability matrix for all pixels into binary file. The matrix size is N*M, where N is total pixels (row order), M is total class.
 

Note that the label definiation of CNN is different from ours 3D mapping, so we need to remap the label definition using change_cnn_label.m, which also provides visualization of binary data.



# Post estimation
Use the standard trajectory output txt from ORB SLAM2, which is N*12 matrix. Each row represents a pose, which is the flattened [R t]
