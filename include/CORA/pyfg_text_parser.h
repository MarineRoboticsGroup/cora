/**
 * @file pyfg_text_parser.h
 * @author
 * @brief Utilities to parse a text file written in the PyFG format
 * @version 0.1
 * @date 2023-10-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>

#include <fstream>
#include <iostream>
#include <string>

namespace CORA {

/**
 * @brief Takes a text file written in the PyFG format and parses it into a
 * CORA::Problem object
 *
 * @param filename the name of the file to parse
 * @return CORA::Problem the parsed problem
 */
CORA::Problem parsePyfgTextToProblem(const std::string &filename);

} // namespace CORA
