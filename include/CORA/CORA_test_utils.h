/**
 * @file CORA_test_utils.h
 * @author Alan Papalia
 * @brief a collection of utility functions for CORA
 * @version 0.1
 * @date 2023-11-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>

#include <string>

namespace CORA {

SparseMatrix readMatrixMarketFile(const std::string &filename);
void writeMatrixMarketFile(const SparseMatrix &A, const std::string &filename);
void printMatrixSparsityPattern(const Matrix &matrix);
std::string getTestDataFpath(const std::string &data_subdir,
                             const std::string &fname);

std::string checkSubmatricesAreCorrect(Problem prob,
                                       const std::string &data_subdir);

} // namespace CORA
