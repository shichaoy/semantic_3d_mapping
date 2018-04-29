// Build
1) Type 'cmake .'
2) Type 'make'

// Run
Usage: stereoslic [options] imageL.png [dispImageL.png [dispImageR.png [imageR.png]]]

option:
   -s: the number of superpixels [default: 1000]
   -i: the number of iterations [default: 10]
   -c: weight of color term [default: 3000]
   -d: weight of disparity term [default: 30]
   -p: penalty value of no disparity plane [default: 3]
   -o: output segment file

Inputs:
   imageL.png: left image
   dispImageL.png: left disparity image (KITTI format: 16bit grayscale multiplied by 256)
   dispImageR.png: right disparity image (KITTI format: 16bit grayscale multiplied by 256)
   imageR.png: right image

Output:
   imageL_slic.png: segment image (16bit grayscale, pixel value = segment index)
   imageL_slic_disparity.png: disparity image (KITTI format: 16bit grayscale multiplied by 256)
   imageL_slic_boundary.png: left image with segment boundary


Example:
1) >./stereoslic 000002_10L.png
   Normal SLIC only using a image
   disparity image is not computed

2) >./stereoslic 000002_10L.png 000002_10L_disparity.png
   StereoSLIC (color(left) + disparity(left))

3) >./stereoslic 000002_10L.png 000002_10L_disparity.png 000002_10R_disparity.png 
   StereoSLIC (color(left) + disparity(left + right))

4) >./stereoslic 000002_10L.png 000002_10L_disparity.png 000002_10R_disparity.png 000002_10R.png
   StereoSLIC (color(left + right) + disparity(left + right))
