
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/CORA_variable_management.h>
#include <CORA/Measurements.h>
#include <CORA/Symbol.h>

namespace CORA {

// Values implementation
void Values::clear() {
  pose_rotations_.clear();
  pose_translations_.clear();
  landmark_positions_.clear();
}

void Values::setPose(const Symbol &sym, const Matrix &R, const Vector &t) {
  pose_rotations_[sym] = R;
  pose_translations_[sym] = t;
}

bool Values::hasPose(const Symbol &sym) const {
  return pose_rotations_.find(sym) != pose_rotations_.end() &&
         pose_translations_.find(sym) != pose_translations_.end();
}

const Matrix *Values::getPoseRotation(const Symbol &sym) const {
  auto it = pose_rotations_.find(sym);
  return it == pose_rotations_.end() ? nullptr : &it->second;
}

const Vector *Values::getPoseTranslation(const Symbol &sym) const {
  auto it = pose_translations_.find(sym);
  return it == pose_translations_.end() ? nullptr : &it->second;
}

void Values::setLandmark(const Symbol &sym, const Vector &p) {
  landmark_positions_[sym] = p;
}

bool Values::hasLandmark(const Symbol &sym) const {
  return landmark_positions_.find(sym) != landmark_positions_.end();
}

const Vector *Values::getLandmark(const Symbol &sym) const {
  auto it = landmark_positions_.find(sym);
  return it == landmark_positions_.end() ? nullptr : &it->second;
}

void updateValuesFromVarMatrix(const Problem &problem, Values &inits,
                               const Matrix &x0) {
  // make sure x0 has the right number of rows and columns
  checkMatrixShape("updateValuesFromVarMatrix",
                   problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());

  Matrix Y = problem.getFormulation() == Formulation::Implicit
                 ? problem.getTranslationExplicitSolution(x0)
                 : x0;

  // Unpack pose rotations and translations
  for (const auto &[sym, idx] : problem.getPoseSymbolMap()) {
    auto pose_vals = extractPose(problem, Y, sym);
    Matrix R = pose_vals.first;
    Vector t = pose_vals.second;
    inits.setPose(sym, R, t);
  }

  // Unpack landmarks
  for (const auto &[sym, idx] : problem.getLandmarkSymbolMap()) {
    Vector p = extractPoint(problem, Y, sym);
    inits.setLandmark(sym, p);
  }
}

void updateVarMatrixRangesBasedOnTranslationVals(const Problem &problem,
                                                 Matrix &x0) {

  checkMatrixShape("updateVarMatrixRangesBasedOnTranslationVals",
                   problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());

  // if implicit formulation, need to convert to explicit first
  Matrix Y = problem.getFormulation() == Formulation::Implicit
                 ? problem.getTranslationExplicitSolution(x0)
                 : x0;

  for (const auto &range_meas : problem.getRangeMeasurements()) {
    Symbol sym1 = range_meas.first_id;
    Symbol sym2 = range_meas.second_id;
    Vector t1 = Y.row(problem.getTranslationIdx(sym1));
    Vector t2 = Y.row(problem.getTranslationIdx(sym2));

    // get the unit-norm bearing vector from t2 to t1. Catch the case where
    // t1 and t2 are the same
    Vector bearing = t2 - t1;
    if (bearing.norm() < 1e-6) {
      bearing = Vector::Random(problem.getRelaxationRank());
    }
    bearing.normalize();
    x0.row(problem.getRangeIdx(range_meas.getSymbolPair())) =
        bearing.transpose();
  }
}

void updateVarMatrixFromValues(const Problem &problem, const Values &inits,
                               Matrix &x0) {
  // make sure x0 has the right number of rows and columns
  checkMatrixShape("updateVarMatrixFromMap", problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());

  bool is_implicit = problem.getFormulation() == Formulation::Implicit;

  // set all of the rotations
  for (const auto &[sym, R] : inits.getAllPoseRotations()) {
    x0.block(problem.getRotationIdx(sym), 0, R.cols(), R.rows()) =
        R.transpose();
  }

  // set all of the translations (if explicit formulation)
  for (const auto &[sym, t] : inits.getAllPoseTranslations()) {
    if (is_implicit)
      break;
    x0.block(problem.getTranslationIdx(sym), 0, 1, t.size()) = t.transpose();
  }

  // set all of the landmarks (if explicit formulation)
  for (const auto &[sym, p] : inits.getAllLandmarkPositions()) {
    if (is_implicit)
      break;
    x0.row(problem.getTranslationIdx(sym)) = p.transpose();
  }

  // set the ranges based on the current translation values
  updateVarMatrixRangesBasedOnTranslationVals(problem, x0);
}

Values getRandomValues(const Problem &problem) {
  Matrix x0 = getRandomVarMatrix(problem);
  Values inits;
  updateValuesFromVarMatrix(problem, inits, x0);
  return inits;
}

Matrix getRandomVarMatrix(const Problem &problem) {
  Matrix x0 = problem.getRandomInitialGuess();
  return x0;
}

Matrix getVarMatrixFromValues(const Problem &problem, const Values &inits) {
  Matrix x0 = getRandomVarMatrix(problem);
  updateVarMatrixFromValues(problem, inits, x0);
  return x0;
}

Values getValuesFromVarMatrix(const Problem &problem, const Matrix &x0) {
  Values inits;
  updateValuesFromVarMatrix(problem, inits, x0);
  return inits;
}

} // namespace CORA