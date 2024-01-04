//
// Created by tim on 11/13/23.
//
#include <CORA/CORA.h>
#include <CORA/CORA_vis.h>

#include <thread> // NOLINT [build/c++11]

int main() {
  CORA::Problem problem =
      CORA::parsePyfgTextToProblem("./bin/data/factor_graph_small.pyfg");
  // "./bin/data/plaza2.pyfg");
  problem.updateProblemData();

  CORA::Matrix x0 = problem.getRandomInitialGuess();

  int max_rank = 10;
  bool verbose = true;
  bool log_iterates = true;
  CORA::CoraResult res;
  res = solveCORA(problem, x0, max_rank, verbose, log_iterates);

  std::cout << "Testing with Random initialization" << std::endl;
  // Visualize the result
  CORA::CORAVis viz{};
  double viz_hz = 10.0;
  // double viz_hz = 2.0;
  viz.run(problem, {res.second}, viz_hz, true);
  return 0;
}
