
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/CORA_variable_management.h>
#include <CORA/Measurements.h>
#include <CORA/Symbol.h>

namespace CORA {

// InitializationMap implementation
void InitializationMap::clear() {
  pose_rotations_.clear();
  pose_translations_.clear();
  landmark_positions_.clear();
}

void InitializationMap::setPoseInitialization(const Symbol &sym,
                                              const Matrix &R,
                                              const Vector &t) {
  pose_rotations_[sym] = R;
  pose_translations_[sym] = t;
}

bool InitializationMap::hasPoseInitialization(const Symbol &sym) const {
  return pose_rotations_.find(sym) != pose_rotations_.end() &&
         pose_translations_.find(sym) != pose_translations_.end();
}

const Matrix *InitializationMap::getPoseRotation(const Symbol &sym) const {
  auto it = pose_rotations_.find(sym);
  return it == pose_rotations_.end() ? nullptr : &it->second;
}

const Vector *InitializationMap::getPoseTranslation(const Symbol &sym) const {
  auto it = pose_translations_.find(sym);
  return it == pose_translations_.end() ? nullptr : &it->second;
}

void InitializationMap::setLandmarkInitialization(const Symbol &sym,
                                                  const Vector &p) {
  landmark_positions_[sym] = p;
}

bool InitializationMap::hasLandmarkInitialization(const Symbol &sym) const {
  return landmark_positions_.find(sym) != landmark_positions_.end();
}

const Vector *InitializationMap::getLandmark(const Symbol &sym) const {
  auto it = landmark_positions_.find(sym);
  return it == landmark_positions_.end() ? nullptr : &it->second;
}

void updateInitMapFromInitMatrix(const Problem &problem,
                                 InitializationMap &inits, const Matrix &x0) {
  // make sure x0 has the right number of rows and columns
  checkMatrixShape("updateInitMapFromInitMatrix", problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());

  if (problem.getFormulation() == Formulation::Implicit) {
    throw NotImplementedException("updateInitMapFromInitMatrix not implemented for "
                                  "translation implicit formulation");
  }

  // Unpack pose rotations and translations
  for (const auto &[sym, idx] : problem.getPoseSymbolMap()) {
    auto pose_vals = extractPose(problem, x0, sym);
    Matrix R = pose_vals.first;
    Vector t = pose_vals.second;
    inits.setPoseInitialization(sym, R, t);
  }

  // Unpack landmarks
  for (const auto &[sym, idx] : problem.getLandmarkSymbolMap()) {
    Vector p = extractPoint(problem, x0, sym);
    inits.setLandmarkInitialization(sym, p);
  }
}

void updateInitMatrixRangesBasedOnTranslationVals(const Problem &problem,
                                                  Matrix &x0) {

  checkMatrixShape("updateInitMatrixRangesBasedOnTranslationVals",
                   problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());
  if (problem.getFormulation() == Formulation::Implicit) {
    throw NotImplementedException(
        "updateInitMatrixRangesBasedOnTranslationVals not implemented for "
        "translation implicit formulation");
  }

  auto ranges = problem.getRangeMeasurements();
  for (const auto &range_meas : ranges) {
    Symbol sym1 = range_meas.first_id;
    Symbol sym2 = range_meas.second_id;
    Vector t1 = x0.row(problem.getTranslationIdx(sym1));
    Vector t2 = x0.row(problem.getTranslationIdx(sym2));

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

void updateInitMatrixFromInitMap(const Problem &problem,
                                 const InitializationMap &inits, Matrix &x0) {
  // make sure x0 has the right number of rows and columns
  checkMatrixShape("updateInitMatrixFromMap", problem.getExpectedVariableSize(),
                   problem.getRelaxationRank(), x0.rows(), x0.cols());

  if (problem.getFormulation() == Formulation::Implicit) {
    throw NotImplementedException(
        "updateInitMatrixFromInitMap not implemented for "
        "translation implicit formulation");
  }

  for (const auto &[sym, p] : inits.getAllLandmarkPositions()) {
    x0.row(problem.getTranslationIdx(sym)) = p.transpose();
  }

  for (const auto &[sym, R] : inits.getAllPoseRotations()) {
    x0.block(problem.getRotationIdx(sym), 0, R.rows(), R.cols()) =
        R.transpose();
  }

  for (const auto &[sym, t] : inits.getAllPoseTranslations()) {
    x0.block(problem.getTranslationIdx(sym), 0, 1, t.size()) = t.transpose();
  }
}

InitializationMap getRandomInitMap(const Problem &problem) {
  Matrix x0 = getRandomInitMatrix(problem);
  InitializationMap inits;
  updateInitMapFromInitMatrix(problem, inits, x0);
  return inits;
}

Matrix getRandomInitMatrix(const Problem &problem) {
  Matrix x0 = problem.getRandomInitialGuess();
  return x0;
}

Matrix getInitMatrixFromInitMap(const Problem &problem,
                                const InitializationMap &inits) {
  Matrix x0 = getRandomInitMatrix(problem);
  updateInitMatrixFromInitMap(problem, inits, x0);
  return x0;
}

InitializationMap getInitMapFromInitMatrix(const Problem &problem,
                                           const Matrix &x0) {
  InitializationMap inits;
  updateInitMapFromInitMatrix(problem, inits, x0);
  return inits;
}

} // namespace CORA