#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>

#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>

void printIfVerbose(bool verbose, std::string msg) {
  if (verbose) {
    std::cout << msg << std::endl;
  }
}

CORA::Scalar thresholdVal(CORA::Scalar val, CORA::Scalar lower_bound,
                          CORA::Scalar upper_bound) {
  if (val < lower_bound) {
    return lower_bound;
  } else if (val > upper_bound) {
    return upper_bound;
  } else {
    return val;
  }
}

namespace CORA {

CoraResult solveCORA(Problem &problem, // NOLINT(runtime/references)
                     const Matrix &x0, int max_relaxation_rank, bool verbose,
                     bool log_iterates, bool show_iterates) {
  // check that x0 has the right number of rows
  if (problem.getFormulation() == Formulation::Explicit) {
    checkMatrixShape("solveCora::Explicit", problem.getDataMatrixSize(),
                     x0.cols(), x0.rows(), x0.cols());
  } else {
    std::cout << "Solving problem in translation implicit mode. Make sure that "
                 "the initial guess only contains rotation and range "
                 "variables."
              << std::endl;
    checkMatrixShape("solveCora::Implicit", problem.rotAndRangeMatrixSize(),
                     x0.cols(), x0.rows(), x0.cols());
  }

  // if log_iterates is true, throw a warning that will be
  // slower than usual
  if (log_iterates) {
    std::cout
        << "WARNING: Logging iterates will slow down the optimization "
           "process.  This is intended for debugging and viz purposes only."
        << std::endl;
  }

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

  // default TNT parameters for CORA
  Optimization::Riemannian::TNTParams<Scalar> params;
  params.Delta0 = 5;
  params.alpha2 = 3.0;
  params.max_TPCG_iterations = 80;
  params.max_iterations = 250;
  params.preconditioned_gradient_tolerance = 1e-6;
  params.gradient_tolerance = 1e-6;
  params.theta = 0.8;
  params.Delta_tolerance = 1e-5;
  params.verbose = show_iterates;
  params.precision = 2;
  params.max_computation_time = 20;
  params.relative_decrease_tolerance = 1e-6;
  params.stepsize_tolerance = 1e-6;
  params.log_iterates = log_iterates;

  // certification parameters
  const Scalar MIN_CERT_ETA = 1e-7;
  const Scalar MAX_CERT_ETA = 1e-1;
  const Scalar REL_CERT_ETA = 5e-6;
  const int LOBPCG_BLOCK_SIZE = 10;
  Scalar eta;

  // metric over the tangent space is the standard matrix trace inner product
  Optimization::Riemannian::RiemannianMetric<Matrix, Matrix, Scalar, Matrix>
      metric =
          [](const Matrix &Y, const Matrix &V1, const Matrix &V2,
             const Matrix &NablaF_Y) { return (V1.transpose() * V2).trace(); };

  // no custom instrumentation function for now
  std::optional<InstrumentationFunction> user_function = std::nullopt;

  CoraTntResult result;
  Matrix X = problem.projectToManifold(x0);
  CertResults cert_results;
  Matrix eigvec_bootstrap;
  std::vector<Matrix> iterates = std::vector<Matrix>();
  bool first_loop = true;
  int loop_cnt = 0;
  while (problem.getRelaxationRank() <= max_relaxation_rank) {
    loop_cnt++;
    // solve the problem
    printIfVerbose(verbose, "\nSolving problem at rank " +
                                std::to_string(problem.getRelaxationRank()));
    result = Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
        f, QM, metric, retract, X, NablaF_Y, precon, params, user_function);
    printIfVerbose(verbose, "Obtained solution with objective value: " +
                                std::to_string(result.f));
    if (log_iterates) {
      for (Matrix iterate : result.iterates) {
        // check that the iterate is the expected size
        checkMatrixShape(
            "solveCora::iterate", problem.getExpectedVariableSize(),
            problem.getRelaxationRank(), iterate.rows(), iterate.cols());
        iterates.push_back(iterate);
      }
    }

    // check if the solution is certified
    eta = thresholdVal(result.f * REL_CERT_ETA, MIN_CERT_ETA, MAX_CERT_ETA);
    if (first_loop) {
      eigvec_bootstrap = result.x;

      // if we are using the translation implicit formulation, we should solve
      // for the translation explicit solution (there is an analytical solution
      // for this)
      if (problem.getFormulation() == Formulation::Implicit) {
        eigvec_bootstrap =
            problem.getTranslationExplicitSolution(eigvec_bootstrap);
      }

    } else {
      eigvec_bootstrap = cert_results.all_eigvecs;
    }

    cert_results = problem.certify_solution(result.x, eta, LOBPCG_BLOCK_SIZE,
                                            eigvec_bootstrap);

    printIfVerbose(
        verbose,
        "Result is certified: " + std::to_string(cert_results.is_certified) +
            " with eta: " + std::to_string(eta) +
            " and theta: " + std::to_string(cert_results.theta));

    // if theta is NaN, then throw an exception
    if (std::isnan(cert_results.theta)) {
      throw std::runtime_error("Theta is NaN");
    }

    // if the solution is certified, we're done
    if (cert_results.is_certified) {
      X = result.x;
      break;
    }

    // otherwise, increment the relaxation rank and try again
    const Scalar SADDLE_GRAD_TOL = 1e-4;
    const Scalar PRECON_SADDLE_GRAD_TOL = 1e-4;
    problem.incrementRank();
    X = saddleEscape(problem, result.x, cert_results.theta, cert_results.x,
                     SADDLE_GRAD_TOL, PRECON_SADDLE_GRAD_TOL);
  }

  // if X has more columns than 'd' then we want to project it down to the
  // correct dimension and refine the solution
  if (X.cols() > problem.dim()) {
    printIfVerbose(verbose, "\nProjecting solution to rank " +
                                std::to_string(problem.dim()) +
                                " and refining.");

    X = projectSolution(problem, X, verbose);

    problem.setRank(problem.dim());
    result = Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
        f, QM, metric, retract, X, NablaF_Y, precon, params, user_function);
    printIfVerbose(verbose, "\nObtained FINAL solution with objective value: " +
                                std::to_string(result.f));

    if (log_iterates) {
      for (Matrix iterate : result.iterates) {
        checkMatrixShape("solveCora::iterate",
                         problem.getExpectedVariableSize(), problem.dim(),
                         iterate.rows(), iterate.cols());

        if (std::getenv("CORA_IGNORE_FINAL_ITERATES") == "true") {
          std::cout << "WARNING - currently not logging final iterates"
                    << std::endl;
        } else {
          iterates.push_back(iterate);
        }
      }
    }

    // let's check if the solution is certified
    std::cout << "Checking certification of refined solution." << std::endl;
    eta = thresholdVal(result.f * REL_CERT_ETA, MIN_CERT_ETA, MAX_CERT_ETA);
    cert_results = problem.certify_solution(result.x, eta, LOBPCG_BLOCK_SIZE,
                                            eigvec_bootstrap);
  }

  // print out whether or not the solution is certified
  printIfVerbose(verbose,
                 "Final solution is certified: " +
                     std::to_string(cert_results.is_certified) +
                     " with eta: " + std::to_string(eta) +
                     " and theta: " + std::to_string(cert_results.theta));

  return std::make_pair(result, iterates);
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
      std::max(16 * alpha_min, 100 * gradient_tolerance / fabs(theta));

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

Matrix projectSolution(const Problem &problem, const Matrix &Y, bool verbose) {
  int d = problem.dim();
  int n = problem.numPoses();
  int l = problem.numLandmarks();
  int r = problem.numRangeMeasurements();
  checkMatrixShape("projectSolution", problem.getExpectedVariableSize(),
                   Y.cols(), Y.rows(), Y.cols());

  // First, compute a thin SVD of Y
  Eigen::JacobiSVD<Matrix> svd(Y, Eigen::ComputeThinU);

  Vector sigmas = svd.singularValues();
  // Construct a diagonal matrix comprised of the first d singular values
  DiagonalMatrix Sigma_d(d);
  DiagonalMatrix::DiagonalVectorType &diagonal = Sigma_d.diagonal();
  for (size_t i = 0; i < d; ++i) {
    diagonal(i) = sigmas(i);
  }

  // print all singular values
  printIfVerbose(verbose, "Singular values of Y: ");
  for (size_t i = 0; i < sigmas.size(); ++i) {
    printIfVerbose(verbose, std::to_string(sigmas(i)));
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

    // if abs(determinant(i)) is far from 1, then print a warning
    if (std::abs(determinants(i) - 1) > 1e-6) {
      // std::cout << "WARNING: Determinant of block " << i
      //           << " is: " << determinants(i) << " not 1" << std::endl;
    }
  }

  printIfVerbose(verbose,
                 "Out of " + std::to_string(n) + " blocks, " +
                     std::to_string(ng0) +
                     " have positive determinant. This is " +
                     std::to_string(static_cast<double>(ng0) / n * 100) +
                     "% of the total.");

  if (n > 0 && ng0 < n / 2) {
    // Less than half of the total number of blocks have the correct sign, so
    // reverse their orientations

    // Get a reflection matrix that we can use to reverse the signs of those
    // blocks of R that have the wrong determinant
    Matrix reflector = Matrix::Identity(d, d);
    reflector(d - 1, d - 1) = -1;

    Yd = Yd * reflector;
  }

  // Project each dxd rotation block to SO(d)
  for (size_t i = 0; i < n; ++i) {
    Yd.block(i * d, 0, d, d) = projectToSOd(Yd.block(i * d, 0, d, d));
  }

  // Project each spherical variable to the unit sphere by normalizing
  // the respective rows from (rot_mat_sz + 1) to (rot_mat_sz + r)
  int rot_mat_sz = problem.numPosesDim();
  Yd.block(rot_mat_sz, 0, r, d).rowwise().normalize();

  problem.checkVariablesAreValid(Yd);

  checkMatrixShape("projectSolution", problem.getExpectedVariableSize(), d,
                   Yd.rows(), Yd.cols());

  // print the singular values of the projected solution
  if (verbose) {
    Eigen::JacobiSVD<Matrix> svd_Yd(Yd, Eigen::ComputeThinU);
    Vector sigmas_Yd = svd_Yd.singularValues();
    printIfVerbose(verbose, "Singular values of Yd: ");
    for (size_t i = 0; i < sigmas_Yd.size(); ++i) {
      printIfVerbose(verbose, std::to_string(sigmas_Yd(i)));
    }
  }

  return Yd;
}

} // namespace CORA
