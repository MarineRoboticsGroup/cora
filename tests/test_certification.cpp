#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/pyfg_text_parser.h>

#include <test_utils.h>

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace CORA {

void checkResultsCertified(CertResults res) {
  // if matrix is PD, then we leave theta and x as 0
  CHECK(res.is_certified);
  CHECK_THAT(res.theta, Catch::Matchers::WithinAbs(0, 1e-6));
  CHECK_THAT(res.x,
             IsApproximatelyEqual<Vector>(Vector::Zero(res.x.size()), 1e-6));
}

void checkNotCertified(CertResults res, Scalar expected_theta,
                       Vector expected_x) {
  CHECK(!res.is_certified);
  CHECK_THAT(res.theta, Catch::Matchers::WithinAbs(expected_theta, 1e-6));

  // need to consider the case where x is the negative of the expected_x
  // since eigenvecs are only unique up to a sign
  CHECK_THAT(res.x, IsApproximatelyEqualUpToSign(expected_x, 1e-6));
}

void printResults(CertResults res) {
  std::cout << "_____________________________________" << std::endl;
  std::cout << "Certified: " << res.is_certified << std::endl;
  std::cout << "Theta: " << res.theta << std::endl;
  std::cout << "x:\n" << res.x << std::endl;
  std::cout << "Num iters: " << res.num_iters << std::endl;
  std::cout << "_____________________________________" << std::endl;
  std::cout << "\n\n\n" << std::endl;
}

void testIdentityMatrixVerification(size_t mat_dim) {
  // for every test run, we will test both interfaces to the 'fast_verification'
  // function: one that takes in a dense matrix representing a block of
  // eigenvectors, and one that takes in a number of eigenvectors to generate
  // a random block of eigenvectors internally
  Matrix I = Matrix::Identity(mat_dim, mat_dim);
  Vector x = Vector::Random(mat_dim).normalized();

  // Identity matrix is PD so should be certified
  checkResultsCertified(fast_verification(I.sparseView(), 0, x));
  checkResultsCertified(fast_verification(I.sparseView(), 0, 1));

  // if we sample a random unit vector 'x' and compute I - xx' then we get a
  // PSD matrix with null space spanned by 'x'
  Matrix xxT = x * x.transpose();
  Matrix I_minus_xxT = I - xxT;

  // when we regularize the matrix with a small eta, it should be certified
  checkResultsCertified(fast_verification(I_minus_xxT.sparseView(), 1e-8, x));
  checkResultsCertified(fast_verification(I_minus_xxT.sparseView(), 1e-8, 1));

  // however, if we compute I - 2*xx' then we get a matrix that is indefinite
  // with a negative eigenpair of (-1, x)
  Matrix I_minus_2xxT = I - 2 * xxT;
  checkNotCertified(fast_verification(I_minus_2xxT.sparseView(), 0, x), -1, x);
  checkNotCertified(fast_verification(I_minus_2xxT.sparseView(), 0, 1), -1, x);
}

TEST_CASE("Test generic verification (small)") {
  testIdentityMatrixVerification(10);
}

TEST_CASE("Test generic verification (large)") {
  testIdentityMatrixVerification(1000);
}

TEST_CASE("Test small RA-SLAM verification") {
  std::string data_subdir = "small_ra_slam_problem";
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  problem.updateProblemData();

  // test that at ground truth the following properties hold:
  // 1. the computed Lambda matrix is all zeros (constraints are not affecting
  // the solution -- this is because the problem is from noiseless data)
  // 2. the solution is certified
  Matrix X_gt = getGroundTruthState(data_subdir);
  Problem::LambdaBlocks lambda_blocks = problem.compute_Lambda_blocks(X_gt);
  CHECK_THAT(lambda_blocks.first, IsApproximatelyEqual<Matrix>(
                                      Matrix::Zero(lambda_blocks.first.rows(),
                                                   lambda_blocks.first.cols()),
                                      1e-6));
  CHECK_THAT(lambda_blocks.second,
             IsApproximatelyEqual<Matrix>(
                 Vector::Zero(lambda_blocks.second.rows()), 1e-6));

  // build the sparse matrix from the lambda blocks and verify that it is
  // still all zeros
  SparseMatrix Lambda =
      problem.compute_Lambda_from_Lambda_blocks(lambda_blocks);
  CHECK_THAT(Lambda, IsApproximatelyEqual<SparseMatrix>(
                         SparseMatrix(Lambda.rows(), Lambda.cols()), 1e-6));

  // verify that the solution is certified (within)
  SparseMatrix S_gt = problem.get_certificate_matrix(X_gt);
  Scalar numerical_tolerance = 1e-6;
  CertResults res_gt = fast_verification(S_gt, numerical_tolerance, 1);
  checkResultsCertified(res_gt);

  // get the random initialization and verify that it is not certified and that
  // the returned theta and x are indeed second order directions of descent
  Matrix X0 = getRandInit(data_subdir);
  SparseMatrix S_rand = problem.get_certificate_matrix(X0);
  CertResults res = fast_verification(S_rand, numerical_tolerance, 1);
  REQUIRE_FALSE(res.is_certified);
  Scalar expected_theta = res.x.transpose() * S_rand * res.x;
  CHECK_THAT(res.theta, Catch::Matchers::WithinAbs(expected_theta, 1e-6));
}

} // namespace CORA
