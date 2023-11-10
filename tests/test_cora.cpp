#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {

void printResults(CoraTntResult res, Problem problem, Matrix X_gt) {
  std::cout << "Cost at GT: " << problem.evaluateObjective(X_gt) << std::endl;
  std::cout << "Egrad norm at GT: " << problem.Euclidean_gradient(X_gt).norm()
            << std::endl;
  Matrix Rgrad = problem.Riemannian_gradient(X_gt);
  std::cout << "Rgrad norm at GT: " << Rgrad.norm() << std::endl;
  if (Rgrad.norm() > 1e-8) {
    std::cout << "Rgrad at GT: " << std::endl << Rgrad << std::endl;
  }

  std::cout << "Run time: " << res.elapsed_time << std::endl;
  std::cout << "Estimated state: " << std::endl << res.x << std::endl;
  std::cout << "Cost: " << res.f << std::endl;
  std::cout << "Outer Iterations: " << res.iterates.size() << std::endl;
  std::cout << "Inner Iterations: " << res.inner_iterations.size() << std::endl;

  std::cout << "Preconditioned gradient norms:" << std::endl;
  for (Scalar val : res.preconditioned_gradient_norms) {
    std::cout << val << "; ";
  }
  std::cout << std::endl;
}

TEST_CASE("Test solve", "[CORA-solve::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  problem.updateProblemData();

  Matrix x0 = problem.getRandomInitialGuess();

  std::string X_gt_fpath = getTestDataFpath(data_subdir, "X_gt.mm");
  Matrix X_gt = readMatrixMarketFile(X_gt_fpath).toDense();

  checkMatrixShape("CORA-solve::small-ra-slam-problem::X_gt",
                   problem.getDataMatrixSize(), 2, X_gt.rows(), X_gt.cols());

  // just check if it runs
  CoraTntResult res = solveCORA(problem, X_gt);
  printResults(res, problem, X_gt);
}
} // namespace CORA
