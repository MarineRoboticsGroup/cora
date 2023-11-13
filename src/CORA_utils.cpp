#include <CORA/CORA_utils.h>

#include <Eigen/CholmodSupport>
#include <Eigen/Geometry>

#include "ILDL/ILDL.h"
#include "Optimization/LinearAlgebra/LOBPCG.h"

namespace CORA {

using SymmetricLinOp =
    Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>;

CertResults fast_verification(const SparseMatrix &S, Scalar eta,
                              const Matrix &X0, size_t max_iters,
                              Scalar max_fill_factor, Scalar drop_tol) {
  // Don't forget to set this on input!
  size_t num_iters = 0;
  Scalar theta = 0;
  Vector x = Vector::Zero(S.rows());

  unsigned int n = S.rows();

  /// STEP 1:  Test positive-semidefiniteness of regularized certificate matrix
  /// M := S + eta * Id via direct factorization

  SparseMatrix Id(n, n);
  Id.setIdentity();
  SparseMatrix M = S + eta * Id;

  /// Test positive-semidefiniteness via direct Cholesky factorization
  Eigen::CholmodSupernodalLLT<SparseMatrix> MChol;

  /// Set various options for the factorization

  // Bail out early if non-positive-semidefiniteness is detected
  MChol.cholmod().quick_return_if_not_posdef = 1;

  // We know that we might be handling a non-PSD matrix, so suppress Cholmod's
  // printed error output
  MChol.cholmod().print = 0;

  // Calculate Cholesky decomposition!
  MChol.compute(M);

  // Test whether the Cholesky decomposition succeeded
  bool PSD = (MChol.info() == Eigen::Success);

  if (!PSD) {
    /// If control reaches here, then lambda_min(S) < -eta, so we must compute
    /// an approximate minimum eigenpair using LOBPCG

    Vector Theta; // Vector to hold Ritz values of S
    Matrix X;     // Matrix to hold eigenvector estimates for S
    size_t num_converged;

    /// Set up matrix-vector multiplication operator with regularized
    /// certificate matrix M

    // Matrix-vector multiplication with regularized certificate matrix M
    SymmetricLinOp Mop = [&M](const Matrix &X) -> Matrix { return M * X; };

    // Custom stopping criterion: terminate as soon as a direction of
    // sufficiently negative curvature is found:
    //
    // x'* S * x < - eta / 2
    //
    Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix> stopfun =
        [&S, eta](size_t i, const SymmetricLinOp &M,
                  const std::optional<SymmetricLinOp> &B,
                  const std::optional<SymmetricLinOp> &T, size_t nev,
                  const Vector &Theta, const Matrix &X, const Vector &r,
                  size_t nc) {
          // Calculate curvature along estimated minimum eigenvector
          Scalar theta = X.col(0).dot(S * X.col(0));
          return (theta < -eta / 2);
        };

    /// STEP 2:  Try computing a minimum eigenpair of M using *unpreconditioned*
    /// LOBPCG.

    // This is a useful computational enhancement for the case in
    // which M has an "obvious" (i.e. well-separated or large-magnitude)
    // negative eigenpair, since in that case LOBPCG permits us to
    // well-approximate this eigenpair *without* the need to construct the
    // preconditioner T

    /// Run preconditioned LOBPCG, using at most 15% of the total allocated
    /// iterations

    double unprecon_iter_frac = .15;
    std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
        Mop, std::optional<SymmetricLinOp>(), std::optional<SymmetricLinOp>(),
        X0, 1, static_cast<size_t>(unprecon_iter_frac * max_iters), num_iters,
        num_converged, 0.0,
        std::optional<
            Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix>>(
            stopfun));

    // Extract eigenvector estimate
    x = X.col(0);

    // Calculate curvature along x
    theta = x.dot(S * x);

    if (theta >= -eta / 2) {
      /// STEP 3:  RUN PRECONDITIONED LOBPCG

      // We did *not* find a direction of sufficiently negative curvature in the
      // allotted number of iterations, so now run preconditioned LOBPCG.  This
      // is most useful for the "hard" cases, in which M has a strictly negative
      // minimum eigenpair that is small-magnitude (i.e. near-zero).

      /// Set up preconditioning operator T

      // Incomplete symmetric indefinite factorization of M

      // Set drop tolerance and max fill factor for ILDL preconditioner
      Preconditioners::ILDLOpts ildl_opts;
      ildl_opts.max_fill_factor = max_fill_factor;
      ildl_opts.drop_tol = drop_tol;

      Preconditioners::ILDL Mfact(M, ildl_opts);

      SymmetricLinOp T = [&Mfact](const Matrix &X) -> Matrix {
        // Preallocate output matrix TX
        Matrix TX(X.rows(), X.cols());

#pragma omp parallel for
        for (unsigned int i = 0; i < X.cols(); ++i) {
          // Calculate TX by preconditioning the columns of X one-by-one
          TX.col(i) = Mfact.solve(X.col(i), true);
        }

        return TX;
      };

      /// Run preconditioned LOBPCG using the remaining alloted LOBPCG
      /// iterations
      std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
          Mop, std::optional<SymmetricLinOp>(),
          std::optional<SymmetricLinOp>(T), X0, 1,
          static_cast<size_t>((1.0 - unprecon_iter_frac) * max_iters),
          num_iters, num_converged, 0.0,
          std::optional<
              Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix>>(
              stopfun));

      // Extract eigenvector estimate
      x = X.col(0);

      // Calculate curvature along x
      theta = x.dot(S * x);

      num_iters += static_cast<size_t>(unprecon_iter_frac * num_iters);
    } // if (!(theta < -eta / 2))
  }   // if(!PSD)

  CertResults results;
  results.is_certified = PSD;
  results.theta = theta;
  results.x = x;
  results.num_iters = num_iters;
  return results;
}

Matrix projectToSOd(const Matrix &M) {
  // Compute the SVD of M
  Eigen::JacobiSVD<Matrix> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Scalar detU = svd.matrixU().determinant();
  Scalar detV = svd.matrixV().determinant();

  if (detU * detV > 0) {
    return svd.matrixU() * svd.matrixV().transpose();
  } else {
    Matrix Uprime = svd.matrixU();
    Uprime.col(Uprime.cols() - 1) *= -1;
    return Uprime * svd.matrixV().transpose();
  }
}

} // namespace CORA
