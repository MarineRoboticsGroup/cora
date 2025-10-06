#include <CORA/CORA.h>
#include <CORA/Measurements.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_variable_management.h>
#include <CORA/Symbol.h>
#include <iostream>

using namespace CORA;

int main(int argc, char **argv) {
  int dim = 3;
  int relax_rank = 6;
  Problem problem(dim, relax_rank);
  problem.setFormulation(Formulation::Implicit);

  // Add two poses and one landmark
  Symbol p0('x', 0);
  Symbol p1('x', 1);
  Symbol l0('l', 0);
  problem.addPoseVariable(p0);
  problem.addPoseVariable(p1);
  problem.addLandmarkVariable(l0);

  // Add priors and measurements
  Matrix R = Matrix::Identity(dim, dim);
  Vector t0 = Vector::Zero(dim);
  Matrix cov = Matrix::Identity(dim, dim) * 1e-2;
  problem.addPosePrior(PosePrior(p0, R, t0, cov));
  problem.addRelativePoseMeasurement(RelativePoseMeasurement(p0, p1, R, Vector::Constant(dim, 0.5), cov));
  problem.addLandmarkPrior(LandmarkPrior(l0, Vector::Constant(dim, 1.0), cov));
  problem.addRangeMeasurement(RangeMeasurement(p1, l0, 1.0, 0.01));

  problem.updateProblemData();

  // random initialization
  Matrix x0 = getRandomVarMatrix(problem);
  std::cout << "x0 shape: " << x0.rows() << " x " << x0.cols() << std::endl;

  // solve
  auto result_pair = solveCORA(problem, x0, problem.getRelaxationRank(), false, false, false);
  auto &cres = result_pair.first;
  std::cout << "Initial solve f = " << cres.f << std::endl;

  // pack solution into Values
  Values vals = getValuesFromVarMatrix(problem, cres.x);
  std::cout << "vals.hasPose p0: " << vals.hasPose(p0) << std::endl;

  // expand problem
  Symbol p2('x', 2);
  Symbol l1('l', 1);
  problem.addPoseVariable(p2);
  problem.addLandmarkVariable(l1);
  problem.addRelativePoseMeasurement(RelativePoseMeasurement(p1, p2, R, Vector::Constant(dim, 0.3), cov));
  problem.addRelativePoseLandmarkMeasurement(RelativePoseLandmarkMeasurement(p2, l1, Vector::Constant(dim, 0.2), cov));
  problem.addRangeMeasurement(RangeMeasurement(p2, l1, 0.5, 0.01));

  problem.updateProblemData();

  // create new x0 from previous values
  Matrix new_x0 = getVarMatrixFromValues(problem, vals);
  std::cout << "new_x0 shape: " << new_x0.rows() << " x " << new_x0.cols() << std::endl;

  // update ranges
  updateVarMatrixRangesBasedOnTranslationVals(problem, new_x0);

  // re-solve
  auto result_pair2 = solveCORA(problem, new_x0, problem.getRelaxationRank(), false, false, false);
  auto &cres2 = result_pair2.first;
  std::cout << "Expanded solve f = " << cres2.f << std::endl;

  return 0;
}
