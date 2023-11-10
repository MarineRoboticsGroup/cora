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

namespace CORA {

using CoraTntResult = Optimization::Riemannian::TNTResult<Matrix, Scalar>;

CoraTntResult solveCORA(const Problem &problem, const Matrix &x0);
inline CoraTntResult solveCORA(std::string filepath) {
  Problem problem = parsePyfgTextToProblem(filepath);
  Matrix x0 = Matrix();
  throw std::runtime_error(
      "Not implemented -- need to decide how to get initialization");
  return solveCORA(problem, x0);
}

} // namespace CORA
