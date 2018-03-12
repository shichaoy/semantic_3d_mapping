
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <Eigen/Core>

#include <scrollgrid/scrollgrid3.hpp>

#include <gtest/gtest.h>

using namespace boost::assign;
using namespace Eigen;
using namespace ca;

TEST(scrollgrid3, hash_roundtrip1) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(200, 100, 400);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        uint64_t hix = grid3.grid_to_hash(gix);
        Vec3Ix gix2 = grid3.hash_to_grid(hix);
        EXPECT_EQ(gix, gix2);
      }
    }
  }

}

TEST(scrollgrid3, hash_roundtrip2) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(21, 31, 41);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        uint64_t hix = grid3.grid_to_hash(gix);
        Vec3Ix gix2 = grid3.hash_to_grid(hix);
        EXPECT_EQ(gix, gix2);
      }
    }
  }

}

TEST(scrollgrid3, scroll1) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(21, 31, 41);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  EXPECT_EQ( grid3.scroll_offset()[0] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[1] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[2] , 0 );

  {
    ca::Vec3Ix gix(0, 0, 1);
    grid3.scroll( gix );
  }

  EXPECT_EQ( grid3.scroll_offset()[0] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[1] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[2] , 1 );

  {
    ca::Vec3Ix gix(0, 0, -1);
    grid3.scroll( gix );
  }

  EXPECT_EQ( grid3.scroll_offset()[0] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[1] , 0 );
  EXPECT_EQ( grid3.scroll_offset()[2] , 0 );


}

TEST(scrollgrid3, scroll2) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(21, 31, 41);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  ca::Vec3Ix gix(2, 0, 0);
  grid3.scroll( gix );

  EXPECT_FLOAT_EQ( grid3.center()[0], center[0]+2*res );

}

TEST(scrollgrid3, scroll3) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(21, 31, 41);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  EXPECT_EQ( grid3.first_i() , 0 );
  EXPECT_EQ( grid3.first_j() , 0 );
  EXPECT_EQ( grid3.first_k() , 0 );

  ca::Vec3Ix gix(2, 0, 0);
  grid3.scroll( gix );

  EXPECT_EQ( grid3.first_i() , 2 );
  EXPECT_EQ( grid3.first_j() , 0 );
  EXPECT_EQ( grid3.first_k() , 0 );

  ca::Vec3Ix gix2(0, -2, 0);
  grid3.scroll( gix2 );

  EXPECT_EQ( grid3.first_i() , 2 );
  EXPECT_EQ( grid3.first_j() , -2 );
  EXPECT_EQ( grid3.first_k() , 0 );


}

TEST(scrollgrid3, scroll4) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(21, 31, 41);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  EXPECT_EQ( grid3.dim_i() , grid3.last_i() - grid3.first_i() );
  EXPECT_EQ( grid3.dim_j() , grid3.last_j() - grid3.first_j() );
  EXPECT_EQ( grid3.dim_k() , grid3.last_k() - grid3.first_k() );

  ca::Vec3Ix gix(2, 0, 0);
  grid3.scroll( gix );

  EXPECT_EQ( grid3.dim_i() , grid3.last_i() - grid3.first_i() );
  EXPECT_EQ( grid3.dim_j() , grid3.last_j() - grid3.first_j() );
  EXPECT_EQ( grid3.dim_k() , grid3.last_k() - grid3.first_k() );

}

TEST(scrollgrid3, grid_to_mem1) {

  Vector3f center(-1, 2, -3);
  Vec3Ix dim(21, 31, 41);
  float res = 0.5;
  ca::ScrollGrid3f grid3( center, dim, res);

#if 0
  std::cerr << "grid3.first_i() = " << grid3.first_i() << std::endl;
  std::cerr << "grid3.first_j() = " << grid3.first_j() << std::endl;
  std::cerr << "grid3.first_k() = " << grid3.first_k() << std::endl;

  std::cerr << "grid3.last_i() = " << grid3.last_i() << std::endl;
  std::cerr << "grid3.last_j() = " << grid3.last_j() << std::endl;
  std::cerr << "grid3.last_k() = " << grid3.last_k() << std::endl;
#endif

  mem_ix_t mix1 = grid3.grid_to_mem( ca::Vec3Ix(2, 1, 4) );
  mem_ix_t mix2 = grid3.grid_to_mem( 2, 1, 4 );

  EXPECT_EQ(mix1, mix2);
}

TEST(scrollgrid3, grid_to_mem2) {

  Vector3f center(-1, 2, -3);
  Vec3Ix dim(21, 31, 41);
  float res = 0.5;
  ca::ScrollGrid3f grid3( center, dim, res);

  std::vector<grid_ix_t> ix;
  ix += 0, 3, 4, 10, 11;


  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        mem_ix_t mix1 = grid3.grid_to_mem_slow( i, j, k);
        mem_ix_t mix2 = grid3.grid_to_mem( i, j, k );
        EXPECT_EQ(mix1, mix2);
      }
    }
  }
}

TEST(scrollgrid3, grid_to_mem3) {

  Vector3f center(-1, 2, -3);
  Vec3Ix dim(21, 31, 41);
  float res = 0.5;
  ca::ScrollGrid3f grid3( center, dim, res);

  std::vector<grid_ix_t> ix;
  ix += 0, 3, 4, 10, 11;


  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        mem_ix_t mix1 = grid3.grid_to_mem_slow( i, j, k);
        mem_ix_t mix2 = grid3.grid_to_mem( i, j, k );
        EXPECT_EQ(mix1, mix2);
      }
    }
  }
}

TEST(scrollgrid3, grid_to_mem4) {

  Vector3f center(-1, 2, -3);
  Vec3Ix dim(21, 31, 41);
  float res = 0.5;
  ca::ScrollGrid3f grid3( center, dim, res);

  ca::Vec3Ix gix( 10, 14,  21);
  mem_ix_t mix = grid3.grid_to_mem(gix);

  grid3.scroll( ca::Vec3Ix( 0, 0, 1) );
  EXPECT_EQ( mix , grid3.grid_to_mem(gix) );
  EXPECT_EQ( mix , grid3.grid_to_mem_slow(gix) );

  grid3.scroll( ca::Vec3Ix( 0, 0, -1) );
  EXPECT_EQ( mix , grid3.grid_to_mem(gix) );
  EXPECT_EQ( mix , grid3.grid_to_mem_slow(gix) );

  grid3.scroll( ca::Vec3Ix( 1, 0, 1) );
  EXPECT_EQ( mix , grid3.grid_to_mem(gix) );
  EXPECT_EQ( mix , grid3.grid_to_mem_slow(gix) );

  grid3.scroll( ca::Vec3Ix( -2, 0, 1) );
  EXPECT_EQ( mix , grid3.grid_to_mem(gix) );
  EXPECT_EQ( mix , grid3.grid_to_mem_slow(gix) );

  grid3.scroll( ca::Vec3Ix( 0, 3, -4) );
  EXPECT_EQ( mix , grid3.grid_to_mem(gix) );
}

TEST(scrollgrid3, grid_to_mem5) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(200, 100, 400);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  grid3.scroll( ca::Vec3Ix(2, 0, 0) );

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        gix += (grid3.scroll_offset() + grid3.radius_ijk());
        mem_ix_t mix = grid3.grid_to_mem_slow(gix);
        mem_ix_t mix2 = grid3.grid_to_mem(gix);
        EXPECT_EQ( mix, mix2 );
      }
    }
  }
}



TEST(scrollgrid3, mem_roundtrip1) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(200, 100, 400);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        gix += (grid3.scroll_offset() + grid3.radius_ijk());
        mem_ix_t mix = grid3.grid_to_mem(gix);
        Vec3Ix gix2 = grid3.mem_to_grid(mix);
        EXPECT_TRUE( gix.cwiseEqual(gix2).all() );

      }
    }
  }
}

TEST(scrollgrid3, mem_roundtrip2) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(200, 100, 400);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  grid3.scroll( ca::Vec3Ix(2, 0, 0) );

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        gix += (grid3.scroll_offset() + grid3.radius_ijk());
        mem_ix_t mix = grid3.grid_to_mem(gix);
        //Vec3Ix gix2 = grid3.mem_to_grid_undo_wrap(mix);
        Vec3Ix gix2 = grid3.mem_to_grid(mix);

        EXPECT_EQ( gix[0], gix2[0] );
        EXPECT_EQ( gix[1], gix2[1] );
        EXPECT_EQ( gix[2], gix2[2] );

      }
    }
  }

}

TEST(scrollgrid3, mem_roundtrip3) {

  Vector3f center(-10, 20, -30);
  Vec3Ix dim(200, 100, 400);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  grid3.scroll( ca::Vec3Ix(202, 0, 0) );

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        gix += (grid3.scroll_offset() + grid3.radius_ijk());
        mem_ix_t mix = grid3.grid_to_mem(gix);
        //Vec3Ix gix2 = grid3.mem_to_grid_undo_wrap(mix);
        Vec3Ix gix2 = grid3.mem_to_grid(mix);
        EXPECT_EQ( gix[0], gix2[0] );
        EXPECT_EQ( gix[1], gix2[1] );
        EXPECT_EQ( gix[2], gix2[2] );
        //EXPECT_TRUE( gix.cwiseEqual(gix2).all() );

      }
    }
  }

}

#if 0
struct TestClearCellsFun : ca::ClearCellsFun {
  TestClearCellsFun(ca::Vec3Ix* last_start, ca::Vec3Ix* last_finish) {
  }
  virtual void operator()(const ca::Vec3Ix& start,
                          const ca::Vec3Ix& finish) const {
    *last_start = start;
    *last_finish = finish;
  }
  ca::Vec3Ix *last_start_, *last_finish_;
};

TEST(scrollgrid3, big_offset1) {
  // when scroll offset is bigger than grid dimensions

  Vector3f center(0.f, 0.f, 0.f);
  Vec3Ix dim(20, 10, 40);
  float res = 0.15;
  ca::ScrollGrid3f grid3( center, dim, res);

  grid3.scroll( ca::Vec3Ix(25, 0, 0) );

  std::vector<grid_ix_t> ix;
  ix += -10, -3, 0, 4, 11;

  BOOST_FOREACH(grid_ix_t i, ix) {
    BOOST_FOREACH(grid_ix_t j, ix) {
      BOOST_FOREACH(grid_ix_t k, ix) {
        Vec3Ix gix(i, j, k);
        gix += (grid3.scroll_offset() + grid3.radius_ijk());
        mem_ix_t mix = grid3.grid_to_mem(gix);
        //Vec3Ix gix2 = grid3.mem_to_grid_undo_wrap(mix);
        Vec3Ix gix2 = grid3.mem_to_grid(mix);
        EXPECT_EQ( gix[0], gix2[0] );
        EXPECT_EQ( gix[1], gix2[1] );
        EXPECT_EQ( gix[2], gix2[2] );
        //EXPECT_TRUE( gix.cwiseEqual(gix2).all() );

      }
    }
  }
}
#endif

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
