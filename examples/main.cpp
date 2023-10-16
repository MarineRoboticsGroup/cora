#include <fstream>
#include <iostream>

#ifdef GPERFTOOLS
#include <gperftools/profiler.h>
#endif

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " [input .g2o file]" << std::endl;
    exit(1);
  }

#ifdef GPERFTOOLS
  ProfilerStart("SE-Sync.prof");
#endif

#ifdef GPERFTOOLS
  ProfilerStop();
#endif
}
