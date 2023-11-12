#include <CORA/CORA.h>
#include <CORA/pyfg_text_parser.h>
#include <test_utils.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>

namespace CORA {

void checkHelperFunctions(std::string data_subdir) {
  Problem prob = getProblem(data_subdir);
  prob.updateProblemData();
  Matrix x0 = getRandInit(data_subdir);

  // check the cost
  Scalar expectedCost = getExpectedCost(data_subdir);
  Scalar cost = prob.evaluateObjective(x0);
  CHECK_THAT(cost, Catch::Matchers::WithinAbs(expectedCost, 1e-6));

  // check Euclidean gradient
  Matrix Egrad = prob.Euclidean_gradient(x0);
  Matrix expectedEgrad = getExpectedEgrad(data_subdir);
  CHECK_THAT(Egrad, IsApproximatelyEqual(expectedEgrad, 1e-6));

  // check Riemannian gradient
  Matrix Rgrad = prob.Riemannian_gradient(x0);
  Matrix expectedRgrad = getExpectedRgrad(data_subdir);
  CHECK_THAT(Rgrad, IsApproximatelyEqual(expectedRgrad, 1e-6));

  //   check the hess-vec product
  Matrix rand_dX = getRandDX(data_subdir);
  Matrix Hvp = prob.Riemannian_Hessian_vector_product(x0, Egrad, rand_dX);
  Matrix expectedHessProd = getExpectedHessProd(data_subdir);
  CHECK_THAT(Hvp, IsApproximatelyEqual(expectedHessProd, 1e-6));
}

TEST_CASE("optimizer helpers RA-SLAM", "[opt-helpers::small_ra_slam_problem]") {
  std::string data_subdir = "small_ra_slam_problem";
  checkHelperFunctions(data_subdir);
}

TEST_CASE("optimizer helpers RPM", "[opt-helpers::single_rpm]") {
  std::string data_subdir = "single_rpm";
  checkHelperFunctions(data_subdir);
}

TEST_CASE("optimizer helpers single range", "[opt-helpers::single_range]") {
  std::string data_subdir = "single_range";
  checkHelperFunctions(data_subdir);
}

// single-rpm cost:

} // namespace CORA
