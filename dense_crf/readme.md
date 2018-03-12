# dense CRF 

This code is based on Philipp Krähenbühl `Efficient Inference in Fully Connected CRFs with Gaussian Edge Potentials`.
http://graphics.stanford.edu/projects/drf/   If you're using this code in a publication, please also cite their papers.


# Running #
This function is called by grid_sensor for 3D CRF. We also provide a 2D CRF example using data under grid_sensor/data_kitti
```bash
rosrun dense_crf comare_hier_dense_crf
```
It takes around 30s to optimize the full 2D KITTI image. Results also saved under test_data/. Sometimes hierarchical and dense CRF might have small difference.

# Changes to the original code

1. Add dense pairwise potential with high order cliques, which can be used for general 2D or 3D CRF optimization.

2. Adapted for CRF 3D grid optimization and superpixel utils.

3. Change to ros catkin.
