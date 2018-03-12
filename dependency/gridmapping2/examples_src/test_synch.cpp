

#include <unistd.h>
#include <iostream>
#include <fstream>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

void proc_write(const char* c) {
  namespace bipc = boost::interprocess;
  //bipc::named_mutex mutex(bipc::create_only, "gm_test_synch");
  bipc::named_mutex mutex(bipc::open_or_create, "gm_test_synch");
  for (int i=0; i < 200; ++i) {
    {
      //bipc::scoped_lock<bipc::named_mutex> lock(mutex);
      std::ofstream out_stream("out.txt", std::fstream::app);
      for (int i=0; i < 100; ++i){
        //std::cout << c;
        out_stream << c;
        out_stream.flush();
        usleep(20000);
      }
      out_stream.close();
    }
    usleep(20000);
  }
}

int main(int argc, char *argv[]) {
  proc_write(argv[1]);
  return 0;
}

