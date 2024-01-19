#pragma once

#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/Symbol.h>

#include <string>
#include <vector>

namespace CORA {

/**
 * @brief This function implements the fast solution verification method
 * (Algorithm 3) described in the paper "Accelerating Certifiable Estimation
 * with Preconditioned Eigensolvers".
 *
 * Given a symmetric sparse matrix S, this function returns a Boolean value
 * indicating whether the regularized matrix M := S + eta * I is
 * positive-semidefinite.  In the event that M is *not* PSD, this function
 * additionally computes a direction of negative curvature x of S, and its
 * associated Rayleight quotient theta := x'Sx < 0, using the LOBPCG method.
 *
 * @param S the sparse matrix to be tested for positive-semidefiniteness
 * @param eta the regularization parameter
 * @param X0 the initial guess for the blocks of eigenvectors to use in LOBPCG
 * @param max_iters the maximum number of LOBPCG iterations
 * @param max_fill_factor the maximum fill factor to use in the incomplete
 * factorization-based preconditioner
 * @param drop_tol the drop tolerance to use in the incomplete
 * factorization-based preconditioner
 * @return the results of the PSD test
 */
CertResults fast_verification(const SparseMatrix &S, Scalar eta,
                              const Matrix &X0, size_t max_iters = 1000,
                              Scalar max_fill_factor = 3,
                              Scalar drop_tol = 1e-3);

/**
 * @brief This function implements the fast solution verification method
 * (Algorithm 3) described in the paper "Accelerating Certifiable Estimation
 * with Preconditioned Eigensolvers".
 *
 * Given a symmetric sparse matrix S, this function returns a Boolean value
 * indicating whether the regularized matrix M := S + eta * I is
 * positive-semidefinite.  In the event that M is *not* PSD, this function
 * additionally computes a direction of negative curvature x of S, and its
 * associated Rayleight quotient theta := x'Sx < 0, using the LOBPCG method.
 *
 * @param S the sparse matrix to be tested for positive-semidefiniteness
 * @param eta the regularization parameter
 * @param nx the block size to use in LOBPCG
 * @param max_iters the maximum number of LOBPCG iterations
 * @param max_fill_factor the maximum fill factor to use in the incomplete
 * factorization-based preconditioner
 * @param drop_tol the drop tolerance to use in the incomplete
 * factorization-based preconditioner
 * @return the results of the PSD test
 */
inline CertResults fast_verification(const SparseMatrix &S, Scalar eta,
                                     size_t nx, size_t max_iters = 1000,
                                     Scalar max_fill_factor = 3,
                                     Scalar drop_tol = 1e-3) {
  return fast_verification(S, eta, Matrix::Random(S.rows(), nx), max_iters,
                           max_fill_factor, drop_tol);
}

Matrix projectToSOd(const Matrix &A);

void saveSolnToTum(const std::vector<Symbol> pose_symbols,
                   const Problem &problem, const Matrix &soln,
                   const std::string &fpath);

} // namespace CORA
