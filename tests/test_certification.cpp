#include <CORA/CORA_problem.h>
#include <CORA/CORA_test_utils.h>
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>

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
  Matrix I = Matrix::Identity(mat_dim, mat_dim);

  // Identity matrix is PD so should be certified
  CertResults identity_res = fast_verification(I.sparseView(), 0, 1);
  checkResultsCertified(identity_res);

  // if we sample a random unit vector 'x' and compute I - xx' then we get a
  // PSD matrix with null space spanned by 'x'
  Vector x = Vector::Random(mat_dim).normalized();
  Matrix xxT = x * x.transpose();
  Matrix I_minus_xxT = I - xxT;

  // when we regularize the matrix with a small eta, it should be certified
  CertResults I_minus_xxT_res_with_reg =
      fast_verification(I_minus_xxT.sparseView(), 1e-8, 1);
  checkResultsCertified(I_minus_xxT_res_with_reg);

  // however, if we compute I - 2*xx' then we get a matrix that is indefinite
  // with a negative eigenpair of (-1, x)
  Matrix I_minus_2xxT = I - 2 * xxT;
  CertResults I_minus_2xxT_res =
      fast_verification(I_minus_2xxT.sparseView(), 0, 1);
  checkNotCertified(I_minus_2xxT_res, -1, x);
}

TEST_CASE("Test generic verification (small)") {
  testIdentityMatrixVerification(10);
}

TEST_CASE("Test generic verification (large)") {
  testIdentityMatrixVerification(1000);
}

} // namespace CORA
