//
// Created by tim on 11/12/23.
//

#pragma once

#ifdef ENABLE_VISUALIZATION

#include <CORA/CORA.h>
#include <CORA/CORA_types.h>
#include <atomic>
#include <memory>
#include <vector>

#include "tonioviz/Visualizer.h"

namespace CORA {
class CORAVis {
public:
  CORAVis();
  void run(const Problem &problem, std::vector<Matrix> iterates, double rate_hz,
           bool verbose = false);
  ~CORAVis() = default;

  std::vector<Matrix>
  projectAndAlignIterates(const Problem &problem,
                          const std::vector<Matrix> &iterates);

private:
  std::atomic<bool> alive{true}; // Shared between threads, will be false if
                                 // either thread terminates

  // pose/point extraction is provided by core CORA helpers

  void dataPlaybackLoop(const std::shared_ptr<mrg::Visualizer> &viz,
                        const Problem &problem, std::vector<Matrix> iterates,
                        double rate_hz, bool verbose);

  void renderLoop(const std::shared_ptr<mrg::Visualizer> &viz);
};

} // namespace CORA

#endif // ENABLE_VISUALIZATION
