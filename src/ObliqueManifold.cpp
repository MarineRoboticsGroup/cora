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

} // namespace CORA
