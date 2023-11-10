#pragma once

#include <CORA/CORA_types.h>

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
 * @param nx the block size to use in LOBPCG
 * @param theta the Rayleigh quotient of the computed negative curvature
 * (returned as a scalar)
 * @param x the computed direction of negative curvature (returned as a vector)
 * @param num_iters the number of iterations LOBPCG executed
 * @param max_iters the maximum number of LOBPCG iterations
 * @param max_fill_factor the maximum fill factor to use in the incomplete
 * factorization-based preconditioner
 * @param drop_tol the drop tolerance to use in the incomplete
 * factorization-based preconditioner
 * @return true
 * @return false
 */
bool fast_verification(const SparseMatrix &S, Scalar eta, size_t nx,
                       Scalar &theta, Vector &x, size_t &num_iters,
                       size_t max_iters = 1000, Scalar max_fill_factor = 3,
                       Scalar drop_tol = 1e-3);

} // namespace CORA
