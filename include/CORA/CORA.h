/** @file
    @brief The main header file for the CORA library.

    This file provides the primary interface to the CORA library. All
    usage of the library should be done through this file.
*/

#pragma once

#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/pyfg_text_parser.h>
#include <string>
#include <utility>
#include <vector>

namespace CORA {

using CoraTntResult = Optimization::Riemannian::TNTResult<Matrix, Scalar>;
using CoraResult = std::pair<CoraTntResult, std::vector<Matrix>>;

CoraResult solveCORA(Problem &problem, const Matrix &x0,
                     int max_relaxation_rank = 20, bool verbose = false,
                     bool log_iterates = false, bool show_iterates = false);
inline CoraResult solveCORA(std::string filepath) {
  Problem problem = parsePyfgTextToProblem(filepath);
  Matrix x0 = Matrix();
  throw std::runtime_error(
      "Not implemented -- need to decide how to get initialization");
  return solveCORA(problem, x0);
}

Matrix saddleEscape(const Problem &problem, const Matrix &Y, Scalar theta,
                    const Vector &v, Scalar gradient_tolerance,
                    Scalar preconditioned_gradient_tolerance);

Matrix projectSolution(const Problem &problem, const Matrix &Y,
                       bool verbose = false);

/**
 * @brief Extract the rotation matrix and translation vector for a pose from
 * a candidate solution. Both returned types use CORA's Matrix/Vector aliases
 * and are sized according to the problem dimension. This keeps the core
 * library dimension-agnostic; any repackaging to SE(3)/SE(2) should be done
 * by consumers (e.g., visualization).
 */
std::pair<Matrix, Vector> extractPose(const Problem &problem,
                                      const Matrix &solution_matrix,
                                      const Symbol &pose_sym);

/**
 * @brief Extract a translation vector for a point from a candidate solution.
 * Returns a CORA::Vector sized to the problem dimension.
 */
Vector extractPoint(const Problem &problem, const Matrix &solution_matrix,
                    const Symbol &point_sym);
} // namespace CORA
