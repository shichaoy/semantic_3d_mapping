
#include "scrollgrid/fixedgrid2.hpp"
#include "scrollgrid/fixedgrid3.hpp"

#include "scrollgrid/scrollgrid2.hpp"
#include "scrollgrid/scrollgrid3.hpp"

#include "scrollgrid/dense_array2.hpp"
#include "scrollgrid/dense_array3.hpp"

#include "scrollgrid/sparse_array3.hpp"
#include "scrollgrid/grid_util.hpp"

#include "scrollgrid/scrolling_strategies.hpp"

#include "scrollgrid/raycasting.hpp"
#include "scrollgrid/occ_raycasting.hpp"

int main(int argc, char *argv[]) {

  ca::ScrollGrid3f grid3( Eigen::Vector3f(0, 0, 0),
                         ca::Vec3Ix(200, 200, 200),
                         0.5 );

  ca::DenseArray3<uint8_t> occ_array3(grid3.dimension());
  ca::bresenham_trace_simple( ca::Vec3Ix(20, 40, 10),
                              ca::Vec3Ix(180, 170, 190),
                              grid3,
                              occ_array3 );


  return 0;
}
