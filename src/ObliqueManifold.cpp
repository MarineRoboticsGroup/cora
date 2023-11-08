#include <CORA/CORA_types.h>
#include <CORA/ObliqueManifold.h>

namespace CORA {

Matrix ObliqueManifold::projectToManifold(const Matrix &A) const {
  // in parallel, normalize each column of A to have unit norm
  Matrix A_normalized = A;
  for (int64_t j = 0; j < A_normalized.cols(); j++) {
    A_normalized.col(j) /= A_normalized.col(j).norm();
  }
  return A_normalized;
}

Matrix ObliqueManifold::projectToTangentSpace(const Matrix &A,
                                              const Matrix &Y) const {
  // get the inner product by taking the row-wise summation of the Hadamard
  // product of Y and A
  Vector inner_prods = (Y.array() * A.array()).colwise().sum();

  // we now want to scale each column of Y by the corresponding inner product
  // and subtract the result from A
  Matrix scaled_cols = Y.array().rowwise() * inner_prods.transpose().array();
  return A - scaled_cols;
}

Matrix ObliqueManifold::retract(const Matrix &Y, const Matrix &V) const {
  return projectToManifold(Y + V);
}

Matrix ObliqueManifold::random_sample(
    const std::default_random_engine::result_type &seed) const {
  // initialize the random number generator
  std::default_random_engine generator(seed);

  // sample each column of the matrix from the standard normal distribution
  std::normal_distribution<Scalar> g;
  Matrix A(r_, n_);
  for (int64_t j = 0; j < A.cols(); j++) {
    for (int64_t i = 0; i < A.rows(); i++) {
      A(i, j) = g(generator);
    }
  }
  return projectToManifold(A);
}

Scalar ObliqueManifold::innerProduct(const Matrix &A, const Matrix &B) const {
  // get the inner product by taking the row-wise summation of the Hadamard
  // product of A and B
  return (A.array() * B.array()).sum();
}

} // namespace CORA
