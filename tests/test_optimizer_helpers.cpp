#include <CORA/CORA.h>
#include <CORA/CORA_test_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <test_utils.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>

namespace CORA {

Problem getProblem(std::string data_subdir) {
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  return problem;
}

Matrix getRandInit(std::string data_subdir) {
  std::string init_path = getTestDataFpath(data_subdir, "X_rand_dim2.mm");
  Matrix x0 = readMatrixMarketFile(init_path).toDense();
  return x0;
}

Matrix getRandDX(std::string data_subdir) {
  std::string rand_dx_path = getTestDataFpath(data_subdir, "rand_dX.mm");
  Matrix rand_dx = readMatrixMarketFile(rand_dx_path).toDense();
  return rand_dx;
}

Scalar getExpectedCost(std::string data_subdir) {
  Scalar expected_cost;
  if (data_subdir == "small_ra_slam_problem") {
    expected_cost = 1.063888372855624e+03;
  } else if (data_subdir == "single_rpm") {
    expected_cost = 0.809173848024762;
  } else if (data_subdir == "single_range") {
    expected_cost = 4.718031199983851;
  } else {
    throw std::runtime_error("Do not have expected cost for: " + data_subdir);
  }
  return expected_cost;
}

Matrix getExpectedEgrad(std::string data_subdir) {
  std::string egrad_path = getTestDataFpath(data_subdir, "expected_egrad.mm");
  Matrix Egrad = readMatrixMarketFile(egrad_path).toDense();
  return Egrad;
}

Matrix getExpectedRgrad(std::string data_subdir) {
  std::string rgrad_path = getTestDataFpath(data_subdir, "expected_rgrad.mm");
  Matrix Rgrad = readMatrixMarketFile(rgrad_path).toDense();
  return Rgrad;
}

Matrix getExpectedHessProd(std::string data_subdir) {
  std::string hess_prod_path = getTestDataFpath(data_subdir, "hessProd.mm");
  Matrix hess_prod = readMatrixMarketFile(hess_prod_path).toDense();
  return hess_prod;
}

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
