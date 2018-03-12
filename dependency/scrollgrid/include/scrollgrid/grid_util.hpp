#ifndef GRID_UTIL_HPP_L137PMBO
#define GRID_UTIL_HPP_L137PMBO

#include <cmath>

#include "scrollgrid/scrollgrid2.hpp"
#include "scrollgrid/scrollgrid3.hpp"

#include "scrollgrid/dense_array2.hpp"
#include "scrollgrid/dense_array3.hpp"

#include "scrollgrid/sparse_array3.hpp"

namespace ca
{

/**
 * Given a scrolling grid3 (maps world ijk to grid ijk), a storage array,
 * and 3 world ijk cuboids, clear area inside the cuboids.
 * Direct approach.
 */
template<class Scalar, class CellType>
void clear_array(const ca::ScrollGrid3<Scalar>& grid3,
                 ca::DenseArray3<CellType>& array,
                 Vec3Ix clear_i_min, Vec3Ix clear_i_max,
                 Vec3Ix clear_j_min, Vec3Ix clear_j_max,
                 Vec3Ix clear_k_min, Vec3Ix clear_k_max) {
  // TODO avoid double-clearing overlaps
  // TODO smarter sweeps
  // TODO adapt to x-fastest

  if (  (clear_i_max - clear_i_min).prod() > 0 ) {
    ROS_ASSERT( grid3.is_inside_grid(clear_i_min) );
    ROS_ASSERT( grid3.is_inside_grid(clear_i_max-Vec3Ix(1,1,1)) );
    for (grid_ix_t i = clear_i_min[0]; i < clear_i_max[0]; ++i) {
      for (grid_ix_t j = clear_i_min[1]; j < clear_i_max[1]; ++j) {
        for (grid_ix_t k = clear_i_min[2]; k < clear_i_max[2]; ++k) {
          //ROS_INFO_STREAM("mem_ix = " << mem_ix);
          mem_ix_t mem_ix = grid3.grid_to_mem(i, j, k);
          array[mem_ix] = CellType();
        }
      }
    }
  }

  if (  (clear_j_max - clear_j_min).prod() > 0 ) {
    ROS_ASSERT( grid3.is_inside_grid(clear_j_min) );
    ROS_ASSERT( grid3.is_inside_grid(clear_j_max-Vec3Ix(1,1,1)) );
    for (grid_ix_t i = clear_j_min[0]; i < clear_j_max[0]; ++i) {
      for (grid_ix_t j = clear_j_min[1]; j < clear_j_max[1]; ++j) {
        for (grid_ix_t k = clear_j_min[2]; k < clear_j_max[2]; ++k) {
          mem_ix_t mem_ix = grid3.grid_to_mem(i, j, k);
          array[mem_ix] = CellType();
        }
      }
    }
  }

  if (  (clear_k_max - clear_k_min).prod() > 0 ) {
    ROS_ASSERT( grid3.is_inside_grid(clear_k_min) );
    ROS_ASSERT( grid3.is_inside_grid(clear_k_max-Vec3Ix(1,1,1)) );
    for (grid_ix_t i = clear_k_min[0]; i < clear_k_max[0]; ++i) {
      for (grid_ix_t j = clear_k_min[1]; j < clear_k_max[1]; ++j) {
        for (grid_ix_t k = clear_k_min[2]; k < clear_k_max[2]; ++k) {
          mem_ix_t mem_ix = grid3.grid_to_mem(i, j, k);
          array[mem_ix] = CellType();
        }
      }
    }
  }

}

template<class Scalar, class CellType>
void clear_array(const ca::ScrollGrid3<Scalar>& grid3,
                 ca::SparseArray3<mem_ix_t>& occ_vox,
                 ca::DenseArray3<CellType>& array,
                 Vec3Ix clear_i_min, Vec3Ix clear_i_max,
                 Vec3Ix clear_j_min, Vec3Ix clear_j_max,
                 Vec3Ix clear_k_min, Vec3Ix clear_k_max) {

  if (  (clear_i_max - clear_i_min).prod() > 0 ) {
    for (SparseArray3<mem_ix_t>::iterator itr = occ_vox.begin();
        itr != occ_vox.end();
        ++itr) {
      const uint64_t& hix(itr->first);
      const mem_ix_t& mix(itr->second);
      Vec3Ix gix(grid3.hash_to_grid(hix));
      if (( (gix.array() >= clear_i_min.array()).all() && (gix.array() < clear_i_max.array()).all() ) ) {
        array[mix] = 0;
        occ_vox.erase(itr);
      }
    }
  }

  if (  (clear_j_max - clear_j_min).prod() > 0 ) {
    for (SparseArray3<mem_ix_t>::iterator itr = occ_vox.begin();
         itr != occ_vox.end();
         ++itr) {
      const uint64_t& hix(itr->first);
      const mem_ix_t& mix(itr->second);
      Vec3Ix gix(grid3.hash_to_grid(hix));
      if (( (gix.array() >= clear_j_min.array()).all() && (gix.array() < clear_j_max.array()).all() ) ) {
        array[mix] = 0;
        occ_vox.erase(itr);
      }
    }
  }

  if (  (clear_k_max - clear_k_min).prod() > 0 ) {
    for (SparseArray3<mem_ix_t>::iterator itr = occ_vox.begin();
         itr != occ_vox.end();
         ++itr) {
      const uint64_t& hix(itr->first);
      const mem_ix_t& mix(itr->second);
      Vec3Ix gix(grid3.hash_to_grid(hix));
      if (( (gix.array() >= clear_k_min.array()).all() && (gix.array() < clear_k_max.array()).all() ) ) {
        array[mix] = 0;
        occ_vox.erase(itr);
      }
    }
  }

}

/**
 * Given a scrolling grid2 (maps world ij to grid ij), a storage array,
 * and 2 world ij cuboids, clear area inside the cuboids.
 * Direct approach.
 */
template<class Scalar, class CellType>
void clear_array2(const ca::ScrollGrid2<Scalar>& grid2,
                  ca::DenseArray2<CellType>& array,
                  Vec2Ix clear_i_min, Vec2Ix clear_i_max,
                  Vec2Ix clear_j_min, Vec2Ix clear_j_max) {
  // TODO avoid double-clearing overlaps
  // TODO smarter sweeps
  // TODO adapt to x-fastest

  if (  (clear_i_max - clear_i_min).prod() > 0 ) {
    ROS_ASSERT(grid2.is_inside_grid(clear_i_min) );
    ROS_ASSERT(grid2.is_inside_grid(clear_i_max-Vec2Ix(1,1)));
    for (grid_ix_t i = clear_i_min[0]; i < clear_i_max[0]; ++i) {
      for (grid_ix_t j = clear_i_min[1]; j < clear_i_max[1]; ++j) {
        //ROS_INFO_STREAM("mem_ix = " << mem_ix);
        mem_ix_t mem_ix = grid2.grid_to_mem(i, j);
        array[mem_ix] = CellType();
      }
    }
  }

  if (  (clear_j_max - clear_j_min).prod() > 0 ) {
    ROS_ASSERT(grid2.is_inside_grid(clear_j_min) );
    ROS_ASSERT(grid2.is_inside_grid(clear_j_max-Vec2Ix(1,1)));
    for (grid_ix_t i = clear_j_min[0]; i < clear_j_max[0]; ++i) {
      for (grid_ix_t j = clear_j_min[1]; j < clear_j_max[1]; ++j) {
        mem_ix_t mem_ix = grid2.grid_to_mem(i, j);
        array[mem_ix] = CellType();
      }
    }
  }

}

} /* ca */

#endif /* end of include guard: GRID_UTIL_HPP_L137PMBO */
