#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>

#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>

namespace CORA {

CoraTntResult solveCORA(Problem &problem, const Matrix &x0,
                        int max_relaxation_rank) {
  // objective function
  Optimization::Objective<Matrix, Scalar, Matrix> f =
      [&problem](const Matrix &Y, const Matrix &NablaF_Y) {
        return problem.evaluateObjective(Y);
      };

  // quadratic model
  Optimization::Riemannian::QuadraticModel<Matrix, Matrix, Matrix> QM =
      [&problem](const Matrix &Y, Matrix &grad,
                 Optimization::Riemannian::LinearOperator<Matrix, Matrix,
                                                          Matrix> &HessOp,
                 Matrix &NablaF_Y) {
        // Compute and cache Euclidean gradient at the current iterate
        NablaF_Y = problem.Euclidean_gradient(Y);

        // Compute Riemannian gradient from Euclidean gradient
        grad = problem.Riemannian_gradient(Y, NablaF_Y);

        // Define linear operator for computing Riemannian Hessian-vector
        // products (cf. eq. (44) in the SE-Sync tech report)
        HessOp = [&problem](const Matrix &Y, const Matrix &Ydot,
                            const Matrix &NablaF_Y) {
          return problem.Riemannian_Hessian_vector_product(Y, NablaF_Y, Ydot);
        };
      };

  // get retraction from problem
  Optimization::Riemannian::Retraction<Matrix, Matrix, Matrix> retract =
      [&problem](const Matrix &Y, const Matrix &V, const Matrix &NablaF_Y) {
        return problem.retract(Y, V);
      };

  // Euclidean gradient (is passed by reference to QM for caching purposes)
  Matrix NablaF_Y;

  // get preconditioner from problem
  std::optional<
      Optimization::Riemannian::LinearOperator<Matrix, Matrix, Matrix>>
      precon = [&problem](const Matrix &Y, const Matrix &Ydot,
                          const Matrix &NablaF_Y) {
        return problem.tangent_space_projection(Y, problem.precondition(Ydot));
      };

  // default TNT parameters
  Optimization::Riemannian::TNTParams<Scalar> params;

  // metric over the tangent space is the standard matrix trace inner product
  Optimization::Riemannian::RiemannianMetric<Matrix, Matrix, Scalar, Matrix>
      metric =
          [](const Matrix &Y, const Matrix &V1, const Matrix &V2,
             const Matrix &NablaF_Y) { return (V1.transpose() * V2).trace(); };

  // no custom instrumentation function for now
  std::optional<InstrumentationFunction> user_function = std::nullopt;

  CoraTntResult result;
  Matrix X = x0;
  while (problem.getRelaxationRank() <= max_relaxation_rank) {
    // solve the problem
    result = Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
        f, QM, metric, retract, X, NablaF_Y, precon, params, user_function);

    // check if the solution is certified
    CertResults cert_results = problem.certify_solution(result.x, 1e-6, 10);

    // if the solution is certified, we're done
    if (cert_results.is_certified) {
      break;
    }

    // otherwise, increment the relaxation rank and try again
    problem.incrementRank();
    Scalar grad_tol = 1e-6;
    Scalar precon_grad_tol = 1e-6;
    X = saddleEscape(problem, result.x, cert_results.theta, cert_results.x,
                     grad_tol, precon_grad_tol);
  }

  // if X has more columns than 'd' then we want to project it down to the
  // correct dimension and refine the solution
  if (X.cols() > problem.dim()) {
    X = projectSolution(problem, X);
    problem.setRank(problem.dim());
    result = Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
        f, QM, metric, retract, X, NablaF_Y, precon, params, user_function);
  }

  // let's check if the solution is certified
  CertResults cert_results = problem.certify_solution(result.x, 1e-6, 10);

  // print out whether or not the solution is certified
  if (!cert_results.is_certified) {
    std::cout << "Warning! Solution is not certified!" << std::endl;
  }

  return result;
}

Matrix saddleEscape(const Problem &problem, const Matrix &Y, Scalar theta,
                    const Vector &v, Scalar gradient_tolerance,
                    Scalar preconditioned_gradient_tolerance) {
  /** v is an eigenvector corresponding to a negative eigenvalue of Q - Lambda,
   * so the KKT conditions for the semidefinite relaxation are not satisfied;
   * this implies that Y is a saddle point of the rank-restricted semidefinite
   * optimization.  Fortunately, v_min can be used to compute a descent
   * direction from this saddle point, as described in Theorem 3.9 of the paper
   * "A Riemannian Low-Rank Method for Optimization over Semidefinite  Matrices
   * with Block-Diagonal Constraints". Define the vector Ydot := e_{r+1} * v';
   * this is a tangent vector to the domain of the SDP and provides a direction
   * of negative curvature */

  // Relaxation rank at the NEXT level of the Riemannian Staircase, i.e. we
  // require that r = Y.cols() + 1
  size_t r = problem.getRelaxationRank();
  if (r != Y.cols() + 1) {
    throw std::runtime_error("Relaxation rank: " + std::to_string(r) +
                             " should be one greater than the number of "
                             "columns in Y: " +
                             std::to_string(Y.cols()) +
                             ". This may happen if the relaxation rank is not "
                             "incremented before attempting saddle escape");
  }

  // Construct the corresponding representation of the saddle point Y in the
  // next level of the Riemannian Staircase by adding a row of 0's
  Matrix Y_augmented = Matrix::Zero(Y.rows(), r);
  Y_augmented.leftCols(r - 1) = Y;

  // Function value at current iterate (saddle point)
  Scalar FY = problem.evaluateObjective(Y_augmented);

  Matrix Ydot = Matrix::Zero(Y.rows(), r);
  Ydot.rightCols<1>() = v;

  // Set the initial step length to the greater of 10 times the distance needed
  // to arrive at a trial point whose gradient is large enough to avoid
  // triggering the gradient norm tolerance stopping condition (according to the
  // local second-order model), or at least 2^4 times the minimum admissible
  // steplength,
  Scalar alpha_min = 1e-6; // Minimum stepsize
  Scalar alpha =
      std::max(16 * alpha_min, 10 * gradient_tolerance / fabs(theta));

  // Vectors of trial stepsizes and corresponding function values
  std::vector<double> alphas;
  std::vector<double> fvals;

  /// Backtracking line search
  Matrix Ytest;
  while (alpha >= alpha_min) {
    // Retract along the given tangent vector using the given stepsize
    Ytest = problem.retract(Y_augmented, alpha * Ydot);

    // Ensure that the trial point Ytest has a lower function value than
    // the current iterate Y, and that the gradient at Ytest is
    // sufficiently large that we will not automatically trigger the
    // gradient tolerance stopping criterion at the next iteration
    Scalar FYtest = problem.evaluateObjective(Ytest);
    Matrix grad_FYtest = problem.Riemannian_gradient(Ytest);
    Scalar grad_FYtest_norm = grad_FYtest.norm();
    Scalar preconditioned_grad_FYtest_norm =
        problem
            .tangent_space_projection(Ytest, problem.precondition(grad_FYtest))
            .norm();

    // Record trial stepsize and function value
    alphas.push_back(alpha);
    fvals.push_back(FYtest);

    if ((FYtest < FY) && (grad_FYtest_norm > gradient_tolerance) &&
        (preconditioned_grad_FYtest_norm > preconditioned_gradient_tolerance)) {
      // Accept this trial point and return success
      return Ytest;
    }
    alpha /= 2;
  }

  // If control reaches here, we failed to find a trial point that satisfied
  // *both* the function decrease *and* gradient bounds.  In order to make
  // forward progress, we will fall back to accepting the trial point that
  // simply minimized the objective value, provided that it strictly *decreased*
  // the objective from the current (saddle) point

  // Find minimum function value from among the trial points
  auto fmin_iter = std::min_element(fvals.begin(), fvals.end());
  auto min_idx = std::distance(fvals.begin(), fmin_iter);

  double f_min = fvals[min_idx];
  double a_min = alphas[min_idx];

  if (f_min < FY) {
    // If this trial point strictly decreased the objective value, accept it and
    // return success
    return problem.retract(Y_augmented, a_min * Ydot);
  } else {
    // NO trial point decreased the objective value: we were unable to escape
    // the saddle point!
    std::cout << "WARNING! BACKTRACKING LINE SEARCH FAILED TO ESCAPE FROM "
                 "SADDLE POINT! (Try decreasing the preconditioned "
                 "gradient norm tolerance)"
              << std::endl;
    return Y_augmented;
  }
}

Matrix projectSolution(const Problem &problem, const Matrix &Y) {
  int d = problem.dim();
  int n = problem.numPoses();
  int l = problem.numLandmarks();
  int r = problem.numRangeMeasurements();

  // First, compute a thin SVD of Y
  Eigen::JacobiSVD<Matrix> svd(Y, Eigen::ComputeThinU);

  Vector sigmas = svd.singularValues();
  // Construct a diagonal matrix comprised of the first d singular values
  DiagonalMatrix Sigma_d(d);
  DiagonalMatrix::DiagonalVectorType &diagonal = Sigma_d.diagonal();
  for (size_t i = 0; i < d; ++i) {
    diagonal(i) = sigmas(i);
  }

  // First, construct a rank-d truncated singular value decomposition for Y
  Matrix Yd = svd.matrixU().leftCols(d) * Sigma_d;
  Vector determinants(n);

  size_t ng0 = 0; // This will count the number of blocks whose
  // determinants have positive sign
  for (size_t i = 0; i < n; ++i) {
    // Compute the determinant of the ith dxd block of Yd
    determinants(i) = Yd.block(i * d, 0, d, d).determinant();
    if (determinants(i) > 0)
      ++ng0;
  }

  if (ng0 < n / 2) {
    // Less than half of the total number of blocks have the correct sign, so
    // reverse their orientations

    // Get a reflection matrix that we can use to reverse the signs of those
    // blocks of R that have the wrong determinant
    Matrix reflector = Matrix::Identity(d, d);
    reflector(d - 1, d - 1) = -1;

    Yd = Yd * reflector;
  }

// Project each dxd rotation block to SO(d)
#pragma omp parallel for
  for (size_t i = 0; i < n; ++i) {
    Yd.block(i * d, 0, d, d) = projectToSOd(Yd.block(i * d, 0, d, d));
  }

  // Project each spherical variable to the unit sphere by normalizing
  // the respective rows
  int rot_mat_sz = problem.numPosesDim();
#pragma omp parallel for
  for (size_t i = 0; i < r; ++i) {
    Yd.block(rot_mat_sz + i, 0, 1, d) /=
        Yd.block(rot_mat_sz + i, 0, 1, d).norm();
  }

  if (problem.getFormulation() == Formulation::Explicit) {
    // Yd already includes the translation estimates (Explicit)
    return Yd;
  } else {
    // form_ == Simplified:  In this case, we also need to recover the
    // optimal translations corresponding to the estimated rotational states
    Matrix X(d, (d + 1) * n);

    // Set rotational states
    X.block(0, 0, rot_mat_sz, d) = Yd;

    // Recover translational states
    throw NotImplementedException(
        "Recovering translational states from rotational states is not yet "
        "implemented");
    // X.block(0, 0, d, n) = recover_translations(B1_, B2_, R);

    return X;
  }
}

} // namespace CORA
