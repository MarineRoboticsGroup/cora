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

Matrix solveCORA(const Problem &problem);
Matrix solveCORA(std::string filepath) {
  Problem problem = parsePyfgTextToProblem(filepath);
  return solveCORA(problem);
}

} // namespace CORA
