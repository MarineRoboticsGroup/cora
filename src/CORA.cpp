#include <CORA/CORA.h>
#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>

namespace CORA {

using CoraTntResult = Optimization::Riemannian::TNTResult<Matrix, Scalar>;

Matrix solveCORA(const Problem &problem) {
  Optimization::Objective<Matrix, Scalar, Matrix> f;
  Optimization::Riemannian::QuadraticModel<Matrix, Matrix, Matrix> QM;
  Optimization::Riemannian::RiemannianMetric<Matrix, Matrix, Scalar, Matrix>
      metric;
  Optimization::Riemannian::Retraction<Matrix, Matrix, Matrix> retract;
  Matrix x0;
  Matrix NablaF_Y;
  std::optional<
      Optimization::Riemannian::LinearOperator<Matrix, Matrix, Matrix>>
      precon;
  Optimization::Riemannian::TNTParams<Scalar> params;
  std::optional<InstrumentationFunction> user_function;

  CoraTntResult result =
      Optimization::Riemannian::TNT<Matrix, Matrix, Scalar, Matrix>(
          f, QM, metric, retract, x0, NablaF_Y, precon, params, user_function);

  return result.x;
}

} // namespace CORA
