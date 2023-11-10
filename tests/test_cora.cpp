#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

namespace CORA {

void printResults(CoraTntResult res, Problem problem, Matrix x0,
                  std::string data_subdir) {
  std::cout << "_____________________________________" << std::endl;
  std::cout << "Data subdir: " << data_subdir << std::endl;
  std::cout << "Cost at init: " << problem.evaluateObjective(x0) << std::endl;
  std::cout << "Egrad norm at init: " << problem.Euclidean_gradient(x0).norm()
            << std::endl;
  Matrix Rgrad = problem.Riemannian_gradient(x0);
  std::cout << "Rgrad norm at init: " << Rgrad.norm() << std::endl;
  std::cout << "Run time: " << res.elapsed_time << std::endl;
  std::cout << "Estimated state: " << std::endl << res.x << std::endl;
  std::cout << "Cost: " << res.f << std::endl;
  std::cout << "Outer Iterations: " << res.iterates.size() << std::endl;
  std::cout << "Inner Iterations: " << res.inner_iterations.size() << std::endl;

  std::cout << "Preconditioned gradient norms:" << std::endl;
  for (Scalar val : res.preconditioned_gradient_norms) {
    std::cout << val << "; ";
  }
  std::cout << "\n_____________________________________" << std::endl;
  std::cout << "\n\n\n" << std::endl;
}

void testScenario(std::string data_subdir) {
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  problem.updateProblemData();

  Matrix x0 = problem.getRandomInitialGuess();

  std::string X_gt_fpath = getTestDataFpath(data_subdir, "X_gt.mm");
  Matrix X_gt = readMatrixMarketFile(X_gt_fpath).toDense();

  checkMatrixShape("CORA-solve::" + data_subdir + "::X_gt",
                   problem.getDataMatrixSize(), 2, X_gt.rows(), X_gt.cols());

  // just check if it runs
  CoraTntResult res = solveCORA(problem, X_gt);
  std::cout << "Testing with Ground Truth initialization" << std::endl;
  printResults(res, problem, X_gt, data_subdir);

  // just check if it runs
  res = solveCORA(problem, x0);
  std::cout << "Testing with Random initialization" << std::endl;
  printResults(res, problem, x0, data_subdir);
}

TEST_CASE("Test solve RA-SLAM", "[CORA-solve::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  testScenario(data_subdir);
}

TEST_CASE("Test solve single-range", "[CORA-solve::single_range]") {
  std::string data_subdir = "single_range";
  testScenario(data_subdir);
}

TEST_CASE("Test solve single-rpm", "[CORA-solve::single_rpm]") {
  std::string data_subdir = "single_rpm";
  testScenario(data_subdir);
}

} // namespace CORA
