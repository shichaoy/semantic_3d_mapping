#ifndef OCC_RAYCASTING_HPP_7K2XI8HT
#define OCC_RAYCASTING_HPP_7K2XI8HT

#include <Eigen/Core>

#include <pcl_util/point_types.hpp>

#include "scrollgrid/grid_types.hpp"
#include "scrollgrid/scrollgrid3.hpp"
#include "scrollgrid/dense_array3.hpp"

#include <math.h> 

namespace ca
{

static const int32_t CA_SG_COMPLETELY_FREE = 0;
static const int32_t CA_SG_COMPLETELY_OCCUPIED = 255;  //250
static const int32_t CA_SG_BELIEF_UPDATE_POS = 20; // when hit
static const int32_t CA_SG_BELIEF_UPDATE_NEG = 10; // when pass-through  2

template<class TraceFunctor>
void occupancy_trace(const Vec3Ix& start_pos,
                     const Vec3Ix& end_pos,
                     const TraceFunctor& fun) {
  // beware: vec3ix are int64_t
  int x = start_pos[0],
      y = start_pos[1],
      z = start_pos[2];
  int dx = end_pos[0] - start_pos[0],
      dy = end_pos[1] - start_pos[1],
      dz = end_pos[2] - start_pos[2];
  int sx, sy, sz;
  //X
  if ( dx>0 ) {
    sx = 1;
  } else if ( dx<0 ) {
    sx = -1;
    dx = -dx;
  } else {
    sx = 0;
  }

  //Y
  if ( dy>0 ) {
    sy = 1;
  } else if ( dy<0 ) {
    sy = -1;
    dy = -dy;
  } else {
    sy = 0;
  }

  //Z
  if ( dz>0 ) {
    sz = 1;
  } else if ( dz<0 ) {
    sz = -1;
    dz = -dz;
  } else {
    sz = 0;
  }

  int ax = 2*dx,
      ay = 2*dy,
      az = 2*dz;

  if ( ( dy <= dx ) && ( dz <= dx ) ) {
    for (int decy=ay-dx, decz=az-dx;
         ;
         x+=sx, decy+=ay, decz+=az) {
      //SetP ( grid,x,y,z,end_pos, atMax, count);
      fun(x,y,z,false);
      //Bresenham step
      if ( x==end_pos[0] ) break;
      if ( decy>=0 ) {
        decy-=ax;
        y+=sy;
      }
      if ( decz>=0 ) {
        decz-=ax;
        z+=sz;
      }
    }
  } else if ( ( dx <= dy ) && ( dz <= dy ) ) {
    //dy>=dx,dy
    for (int decx=ax-dy,decz=az-dy;
         ;
         y+=sy,decx+=ax,decz+=az ) {
      // SetP ( grid,x,y,z,end_pos, atMax, count);
      fun(x,y,z,false);
      //Bresenham step
      if ( y==end_pos[1] ) break;
      if ( decx>=0 ) {
        decx-=ay;
        x+=sx;
      }
      if ( decz>=0 ) {
        decz-=ay;
        z+=sz;
      }
    }
  } else if ( ( dx <= dz ) && ( dy <= dz ) ) {
    //dy>=dx,dy
    for (int decx=ax-dz,decy=ay-dz;
         ;
         z+=sz,decx+=ax,decy+=ay ) {
      //SetP ( grid,x,y,z,end_pos, atMax, count);
      fun(x,y,z,false);
      //Bresenham step
      if ( z==end_pos[2] ) break;
      if ( decx>=0 ) {
        decx-=az;
        x+=sx;
      } if ( decy>=0 ) {
        decy-=az;
        y+=sy;
      }
    }
  }
  fun(x,y,z,true);
}

template<class TraceFunctor>
void occupancy_trace_dist(const Vec3Ix& start_pos,
                     const Vec3Ix& end_pos,
                     const TraceFunctor& fun,
		     double distance) {  //TODO add distance
  // beware: vec3ix are int64_t
  int x = start_pos[0],
      y = start_pos[1],
      z = start_pos[2];
  int dx = end_pos[0] - start_pos[0],
      dy = end_pos[1] - start_pos[1],
      dz = end_pos[2] - start_pos[2];
  int sx, sy, sz;
  //X
  if ( dx>0 ) {
    sx = 1;
  } else if ( dx<0 ) {
    sx = -1;
    dx = -dx;
  } else {
    sx = 0;
  }

  //Y
  if ( dy>0 ) {
    sy = 1;
  } else if ( dy<0 ) {
    sy = -1;
    dy = -dy;
  } else {
    sy = 0;
  }

  //Z
  if ( dz>0 ) {
    sz = 1;
  } else if ( dz<0 ) {
    sz = -1;
    dz = -dz;
  } else {
    sz = 0;
  }

  int ax = 2*dx,
      ay = 2*dy,
      az = 2*dz;

  if ( ( dy <= dx ) && ( dz <= dx ) ) {
    for (int decy=ay-dx, decz=az-dx;
         ;
         x+=sx, decy+=ay, decz+=az) {
      //SetP ( grid,x,y,z,end_pos, atMax, count);
//       fun(x,y,z,false,distance);
      distance=sqrt((x-start_pos[0])*(x-start_pos[0])+(y-start_pos[1])*(y-start_pos[1])+(z-start_pos[2])*(z-start_pos[2]));
      fun(x,y,z,false,distance);
      //Bresenham step
      if ( x==end_pos[0] ) break;
      if ( decy>=0 ) {
        decy-=ax;
        y+=sy;
      }
      if ( decz>=0 ) {
        decz-=ax;
        z+=sz;
      }
    }
  } else if ( ( dx <= dy ) && ( dz <= dy ) ) {
    //dy>=dx,dy
    for (int decx=ax-dy,decz=az-dy;
         ;
         y+=sy,decx+=ax,decz+=az ) {
      // SetP ( grid,x,y,z,end_pos, atMax, count);
//       fun(x,y,z,false,distance);
      distance=sqrt((x-start_pos[0])*(x-start_pos[0])+(y-start_pos[1])*(y-start_pos[1])+(z-start_pos[2])*(z-start_pos[2]));
      fun(x,y,z,false,distance);
      //Bresenham step
      if ( y==end_pos[1] ) break;
      if ( decx>=0 ) {
        decx-=ay;
        x+=sx;
      }
      if ( decz>=0 ) {
        decz-=ay;
        z+=sz;
      }
    }
  } else if ( ( dx <= dz ) && ( dy <= dz ) ) {
    //dy>=dx,dy
    for (int decx=ax-dz,decy=ay-dz;
         ;
         z+=sz,decx+=ax,decy+=ay ) {
      //SetP ( grid,x,y,z,end_pos, atMax, count);
//       fun(x,y,z,false,distance);
      distance=sqrt((x-start_pos[0])*(x-start_pos[0])+(y-start_pos[1])*(y-start_pos[1])+(z-start_pos[2])*(z-start_pos[2]));
      fun(x,y,z,false,distance);
      //Bresenham step
      if ( z==end_pos[2] ) break;
      if ( decx>=0 ) {
        decx-=az;
        x+=sx;
      } if ( decy>=0 ) {
        decy-=az;
        y+=sy;
      }
    }
  }
//       fun(x,y,z,true,distance);
      distance=sqrt((x-start_pos[0])*(x-start_pos[0])+(y-start_pos[1])*(y-start_pos[1])+(z-start_pos[2])*(z-start_pos[2]));
      fun(x,y,z,true,distance);
}
/**
 * Update occupancy information along ray.
 */
template<class GridScalar>
void occupancy_trace_simple(const Vec3Ix& start_pos, // in ijk
                            const Vec3Ix& end_pos, // in ijk
                            const ca::ScrollGrid3<GridScalar>& grid3,
                            ca::DenseArray3<uint8_t>& array3)
                            {
  //int ray_ctr = 0;
  // beware: vec3ix are int64_t
  int x = start_pos[0],
      y = start_pos[1],
      z = start_pos[2];
  int dx = end_pos[0] - start_pos[0],
      dy = end_pos[1] - start_pos[1],
      dz = end_pos[2] - start_pos[2];
  int sx, sy, sz;
  //X
  if ( dx>0 ) {
    sx = 1;
  } else if ( dx<0 ) {
    sx = -1;
    dx = -dx;
  } else {
    sx = 0;
  }

  //Y
  if ( dy>0 ) {
    sy = 1;
  } else if ( dy<0 ) {
    sy = -1;
    dy = -dy;
  } else {
    sy = 0;
  }

  //Z
  if ( dz>0 ) {
    sz = 1;
  } else if ( dz<0 ) {
    sz = -1;
    dz = -dz;
  } else {
    sz = 0;
  }

  int ax = 2*dx,
      ay = 2*dy,
      az = 2*dz;

  if ( ( dy <= dx ) && ( dz <= dx ) ) {
    // this is tracing loop. we step through the ray in integer coordinates
    // until we reach the end
    for (int decy=ay-dx, decz=az-dx;
         ;
         x+=sx, decy+=ay, decz+=az) {

      // here we are in the middle of tracing the
      // ray. here we are passing through.
      mem_ix_t mem_ix = grid3.grid_to_mem(x, y, z);

      // by definition here we are passing through
      // TODO correct & efficent boundary check
      int32_t new_value = static_cast<int32_t>(array3[mem_ix])-CA_SG_BELIEF_UPDATE_NEG;
      array3[mem_ix] = static_cast<uint8_t>(std::max(CA_SG_COMPLETELY_FREE, new_value));

      //Bresenham step
      if ( x==end_pos[0] ) break;
      if ( decy>=0 ) {
        decy-=ax;
        y+=sy;
      }
      if ( decz>=0 ) {
        decz-=ax;
        z+=sz;
      }
    }
  } else if ( ( dx <= dy ) && ( dz <= dy ) ) {
    //dy>=dx,dy
    for (int decx=ax-dy,decz=az-dy;
         ;
         y+=sy,decx+=ax,decz+=az ) {

      mem_ix_t mem_ix = grid3.grid_to_mem(x, y, z);
      int32_t new_value = static_cast<int32_t>(array3[mem_ix])-CA_SG_BELIEF_UPDATE_NEG;
      array3[mem_ix] = static_cast<uint8_t>(std::max(CA_SG_COMPLETELY_FREE, new_value));

      //Bresenham step
      if ( y==end_pos[1] ) break;
      if ( decx>=0 ) {
        decx-=ay;
        x+=sx;
      }
      if ( decz>=0 ) {
        decz-=ay;
        z+=sz;
      }
    }
  } else if ( ( dx <= dz ) && ( dy <= dz ) ) {
    //dy>=dx,dy
    for (int decx=ax-dz,decy=ay-dz;
         ;
         z+=sz,decx+=ax,decy+=ay ) {

      mem_ix_t mem_ix = grid3.grid_to_mem(x, y, z);
      int32_t new_value = static_cast<int32_t>(array3[mem_ix])-CA_SG_BELIEF_UPDATE_NEG;
      array3[mem_ix] = static_cast<uint8_t>(std::max(CA_SG_COMPLETELY_FREE, new_value));

      //array3[mem_ix] = ray_ctr++;
      //Bresenham step
      if ( z==end_pos[2] ) break;
      if ( decx>=0 ) {
        decx-=az;
        x+=sx;
      } if ( decy>=0 ) {
        decy-=az;
        y+=sy;
      }
    }
  }

  // here we are at the end of the ray.
  // here we update according to xyz

  mem_ix_t mem_ix = grid3.grid_to_mem(x, y, z);
  int32_t new_value = static_cast<int32_t>(array3[mem_ix])+CA_SG_BELIEF_UPDATE_POS;
  array3[mem_ix] = static_cast<uint8_t>(std::min(CA_SG_COMPLETELY_OCCUPIED, new_value));

}
} /* ca
 */

#endif /* end of include guard: OCC_RAYCASTING_HPP_7K2XI8HT */

