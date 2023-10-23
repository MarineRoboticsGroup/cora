/**
 * @file CORA_preconditioners.h
 * @author
 * @brief
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_types.h>

#include "Optimization/Riemannian/Concepts.h"
#include "Preconditioners/LSChol/include/LSChol/LSChol.h"

namespace CORA {

/**
 * @brief Generate a block-diagonal preconditioner for the given matrix A based
 * on Cholesky decompositions of each block. This assumes that the matrix A is
 * the un-marginalized data matrix, so the last row/column of A is ignored. When
 * applying the preconditioner, the last row/column of the input vector is
 * mapped to zero. In SLAM, this corresponds to pinning the last translation
 * to the origin.
 *
 * @param A the matrix to precondition (should be symmetric positive definite)
 * @param block_sizes the sizes of the blocks (should sum to A.rows() - 1)
 * @return Optimization::Riemannian::LinearOperator<Matrix, Matrix>
 */
Optimization::Riemannian::LinearOperator<Matrix, Matrix>
constructCoraBlockCholeskyPreconditioner(const SparseMatrix &A,
                                         const Vector &block_sizes) {
  // TODO(magoun): implement this function
  throw NotImplementedException();

  /**
   * @brief compute Cholesky decomposition with permutation ordering to minimize
   * fill-in store the Cholesky factor and the permutation return a
   * LinearOperator that applies the block-wise Cholesky factors
   */
}

/**
 * @brief Similar to constructCoraBlockCholeskyPreconditioner, but uses the
 * marginalized data matrix, so does not ignore the last row/column of A
 *
 * @param A the matrix to precondition (should be symmetric positive definite)
 * @param block_sizes the sizes of the blocks (should sum to A.rows())
 * @return Optimization::Riemannian::LinearOperator<Matrix, Matrix>
 */
Optimization::Riemannian::LinearOperator<Matrix, Matrix>
constructMarginalizedCoraBlockCholeskyPreconditioner(
    const SparseMatrix &A, const Vector &block_sizes) {
  // TODO(magoun): implement this function
  throw NotImplementedException();

  /**
   * @brief compute Cholesky decomposition with permutation ordering to minimize
   * fill-in store the Cholesky factor and the permutation return a
   * LinearOperator that applies the block-wise Cholesky factors
   */
}

} // namespace CORA
