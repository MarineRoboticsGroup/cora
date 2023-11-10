#include <CORA/CORA.h>
#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>

namespace CORA {

CoraTntResult solveCORA(const Problem &problem, const Matrix &x0) {
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

  // Euclidean gradient (not used)
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

  CoraTntResult result =
      Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
          f, QM, metric, retract, x0, NablaF_Y, precon, params, user_function);

  return result;
}

} // namespace CORA
