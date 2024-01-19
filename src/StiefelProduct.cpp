
#include <Eigen/QR>
#include <Eigen/SVD>

#include "CORA/StiefelProduct.h"
namespace CORA {

Matrix StiefelProduct::projectToManifold(const Matrix &A) const {
  // We use a generalization of the well-known SVD-based projection for the
  // orthogonal and special orthogonal groups; see for example Proposition 7
  // in the paper "Projection-Like Retractions on Matrix
  // Manifolds" by Absil and Malick.

  Matrix P(p_, k_ * n_);

  // check that A is the correct size
  if (A.rows() != p_ || A.cols() != k_ * n_) {
    throw std::runtime_error("Error in StiefelProduct::projectToManifold: "
                             "Shape of A is: " +
                             std::to_string(A.rows()) + " x " +
                             std::to_string(A.cols()) +
                             " but should be: " + std::to_string(p_) + " x " +
                             std::to_string(k_ * n_));
  }

  for (auto i = 0; i < n_; ++i) {
    auto start_col = static_cast<Index>(i * k_);
    // Compute the (thin) SVD of the ith block of A
    Eigen::JacobiSVD<Matrix> SVD(A.block(0, start_col, p_, k_),
                                 Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Set the ith block of P to the SVD-based projection of the ith block of A
    P.block(0, start_col, p_, k_) = SVD.matrixU() * SVD.matrixV().transpose();
  }
  return P;
}

Matrix StiefelProduct::SymBlockDiagProduct(const Matrix &A, const Matrix &BT,
                                           const Matrix &C) const {
  // Preallocate result matrix
  Matrix R(p_, k_ * n_);
  Matrix P(k_, k_);
  Matrix S(k_, k_);

  for (auto i = 0; i < n_; ++i) {
    auto start_col = static_cast<Index>(i * k_);
    // Compute block product Bi' * Ci
    P = BT.block(start_col, 0, k_, p_) * C.block(0, start_col, p_, k_);
    // Symmetrize this block
    S = .5 * (P + P.transpose());
    // Compute Ai * S and set corresponding block of R
    R.block(0, start_col, p_, k_) = A.block(0, start_col, p_, k_) * S;
  }
  return R;
}

Matrix StiefelProduct::random_sample(
    const std::default_random_engine::result_type &seed) const {
  // Generate a matrix of the appropriate dimension by sampling its elements
  // from the standard Gaussian
  std::default_random_engine generator(seed);
  std::normal_distribution<Scalar> g;

  Matrix R(p_, k_ * n_);
  for (size_t r = 0; r < p_; ++r)
    for (size_t c = 0; c < k_ * n_; ++c)
      R(static_cast<Index>(r), static_cast<Index>(c)) = g(generator);
  return projectToManifold(R);
}
} // namespace CORA
