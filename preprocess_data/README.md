
We explain preprocessing steps more details here. Welcome to contact if you have problems running them.

### Disparity/Depth
We use [Elas](http://www.cvlibs.net/software/libelas/) to compute dense disparity and depth. Ros ELas is preferred due to easy interface. Save the depth image (mm unit) in CV_16UC1 format.


### Superpixel
We use [SLIC](http://www.cvlibs.net/projects/displets/), external/stereoslic. A complete copy with modification is in this repo ```/preprocess_data/code/superpixel``` We just change the interface function in main.cpp.  We find single image to be more robust than stereo, even it is called stereoslic. Compile by ``` cmake . &&  make ```

Then ``` ./stereoslic aa.png -s 150 ``` 150 is superpixel num. It will read sample images in ```raw_imgs/``` and save an image to ```superpixel_ind/``` with the same size as raw image, representing each pixel's superpixel index. To efficiently find the containing pixels of one superpixel in the latter c++ code, run ```superpixel_img2bin.m``` to change superpixel image into special binary data. The script also provides visualization of binary data to check whether it is correct. The general idea is to save a 170x2077 matrix, where 170 is maximum number of superpixel, each row represents one superpixel and its containing pixel location in this format:  [ num_of_triples*3, [row1, start_col1, end_col1], [row2, start_col2, end_col2]... ]  Each triple represents a range of pixels in one row, note that row1 might ```=``` row2.   

If you want to increase superpixel num (>170), change ```max_sp_num``` in  ```superpixel_img2bin.m``` and also c++ ```read_superpixel_bin()``` in ```dense_crf/libs/superpixel.cpp```


### CNN 2D
We use [Dilation Network](https://github.com/fyu/dilation) to predict the 2D semantic segmentation, which can be replaced by other segmentation. Our modified prediction code is under ```/preprocess_data/code/cnn_2d/predict_dilanet.py```  Change caffe path at the top and dataset path in batch_predict_kitti().  It will save the class probability matrix for all pixels into binary file. The matrix size is N*M, where N is total pixels (row order), M is total class.

Note that the label definiation of dilatnetCNN is different from ours 3D CRF, so we need to remap the label using ```change_cnn_label.m```  It also provides visualization of binary data.


### Post estimation
Use the standard trajectory output txt from [ORB SLAM2](https://github.com/raulmur/ORB_SLAM2), which is N*12 matrix. Each row represents a pose, which is the flattened [R t] matrix.
