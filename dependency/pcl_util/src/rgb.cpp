/**
 * @author  Daniel Maturana
 * @year    2015
 *
 * @attention Copyright (c) 2015
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 **@=*/



#include <iostream>
#include <pcl_util/pcl_util.hpp>

int main(int argc, char *argv[]) {
  float rgb = ca::pcl_util::int_to_pcl_rgb(49, 48, 49);
  std::cout << "rgb = " << rgb << "\n";

  return 0;
}
