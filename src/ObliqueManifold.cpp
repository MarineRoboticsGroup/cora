#include <CORA/CORA_types.h>
#include <CORA/ObliqueManifold.h>

namespace CORA {

Matrix ObliqueManifold::projectToManifold(const Matrix &A) const {
  // check that the dimensions of A are as expected
  checkMatrixShape("ObliqueManifold::projectToManifold", r_, n_, A.rows(),
                   A.cols());

  // in parallel, normalize each column of A to have unit norm
  Matrix A_normalized = A.colwise().normalized();
  return A_normalized;
}

Matrix ObliqueManifold::projectToTangentSpace(const Matrix &Y,
                                              const Matrix &V) const {
  // get the inner product by taking the row-wise summation of the Hadamard
  // product of Y and A
  Vector inner_prods = (Y.array() * V.array()).colwise().sum();

  // we now want to scale each column of Y by the corresponding inner product
  // and subtract the result from A
  Matrix scaled_cols = Y.array().rowwise() * inner_prods.transpose().array();

  return V - scaled_cols;
}

Matrix ObliqueManifold::random_sample(
    const std::default_random_engine::result_type &seed) const {
  // initialize the random number generator
  std::default_random_engine generator(seed);

  // sample each column of the matrix from the standard normal distribution
  std::normal_distribution<Scalar> g;
  Matrix A(r_, n_);
  for (auto j = 0; j < A.cols(); j++) {
    for (auto i = 0; i < A.rows(); i++) {
      A(i, j) = g(generator);
    }
  }
  return projectToManifold(A);
}

} // namespace CORA
