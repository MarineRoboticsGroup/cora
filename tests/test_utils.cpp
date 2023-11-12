
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/pyfg_text_parser.h>
#include <test_utils.h>

#include <unsupported/Eigen/SparseExtra>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

/*********   PROBLEM SETUP   ***********/

namespace CORA {

SparseMatrix readMatrixMarketFile(const std::string &filename) {
  SparseMatrix A;
  bool is_symmetric;
  // read the first line of the file to see if it is symmetric
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Could not open file");
  }

  // Read the first line of the file
  std::string line;
  if (std::getline(file, line)) {
    // Check if the line contains the word "symmetric"
    if (line.find("symmetric") != std::string::npos) {
      is_symmetric = true;
    } else {
      is_symmetric = false;
    }
  } else {
    throw std::runtime_error("Could not read first line of file");
  }

  Eigen::loadMarket(A, filename);

  if (is_symmetric) {
    return A.selfadjointView<Eigen::Lower>();
  }

  return A;
}

void writeMatrixMarketFile(const SparseMatrix &A, const std::string &filename) {
  Eigen::saveMarket(A, filename);
}

void printMatrixSparsityPattern(const Matrix &matrix) {
  // if entry is zero- print "-", otherwise print "X"
  // make sure that vertical spacing is the same as horizontal spacing
  std::cout << "Matrix sparsity pattern: " << std::endl;

  auto rows = matrix.rows();
  auto cols = matrix.cols();

  // Calculate the maximum width needed for any matrix element to ensure
  // consistent spacing
  int max_width =
      std::max(1, static_cast<int>(std::log10(matrix.maxCoeff())) + 1);

  // Print the matrix with proper spacing
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (matrix(i, j) != 0) {
        std::cout << "X";
      } else {
        std::cout << "-";
      }

      // Add padding spaces to ensure identical vertical and horizontal spacing
      for (int k = 0; k < max_width; ++k) {
        std::cout << " ";
      }

      // Add a space between columns
      if (j < cols - 1) {
        std::cout << " ";
      }
    }
    std::cout << std::endl;
  }
}

std::string getTestDataFpath(const std::string &data_subdir,
                             const std::string &fname) {
  std::string curr_path = std::filesystem::current_path();
  std::string filepath =
      std::filesystem::path(curr_path) / "./bin/data" / data_subdir / fname;
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error(
        "File does not exist: " + filepath +
        "\nThis may be because you are running the tests from the wrong" +
        " directory. We expect to be running from <repo_root>/build");
  }
  return filepath;
}

std::string checkSubmatricesAreCorrect(Problem prob,
                                       const std::string &data_subdir) {
  CoraDataSubmatrices data_submatrices = prob.getDataSubmatrices();

  std::string error_msg;

  // map from file name to the sub matrix
  std::map<std::string, SparseMatrix> submatrices = {
      {"Arange.mm", data_submatrices.range_incidence_matrix},
      {"OmegaRange.mm", data_submatrices.range_precision_matrix},
      {"RangeDistances.mm", data_submatrices.range_dist_matrix},
      {"Apose.mm", data_submatrices.rel_pose_incidence_matrix},
      {"OmegaPose.mm", data_submatrices.rel_pose_translation_precision_matrix},
      {"T.mm", data_submatrices.rel_pose_translation_data_matrix},
      {"RotConLaplacian.mm", data_submatrices.rotation_conn_laplacian},
      {"DataMatrix.mm", prob.getDataMatrix()}};

  for (auto &submatrix : submatrices) {
    std::string filepath = getTestDataFpath(data_subdir, submatrix.first);

    SparseMatrix expected_submatrix = readMatrixMarketFile(filepath);
    SparseMatrix actual_submatrix = submatrix.second;

    // if expected matrix has no rows/cols, then we just want to make sure that
    // the actual matrix either has zero rows or zero cols
    if (expected_submatrix.rows() == expected_submatrix.cols() &&
        expected_submatrix.rows() == 0) {
      if (actual_submatrix.rows() != 0 && actual_submatrix.cols() != 0) {
        error_msg += "Submatrix " + submatrix.first + " has " +
                     std::to_string(actual_submatrix.rows()) + " rows and " +
                     std::to_string(actual_submatrix.cols()) +
                     " cols but should have 0 rows or 0 cols\n";
      }
      continue;
    }

    if (expected_submatrix.rows() != actual_submatrix.rows()) {
      // check if matrix has the wrong number of rows
      error_msg += "Submatrix " + submatrix.first + " has " +
                   std::to_string(actual_submatrix.rows()) +
                   " rows but should have " +
                   std::to_string(expected_submatrix.rows()) + "\n";
    } else if (expected_submatrix.cols() != actual_submatrix.cols()) {
      // check if matrix has the wrong number of cols
      error_msg += "Submatrix " + submatrix.first + " has " +
                   std::to_string(actual_submatrix.cols()) +
                   " cols but should have " +
                   std::to_string(expected_submatrix.cols()) + "\n";
    } else if (!expected_submatrix.isApprox(actual_submatrix)) {
      // check if matrix has the wrong values
      error_msg += "Submatrix " + submatrix.first + " is incorrect\n";

      // //* Debugging
      // SparseMatrix diff = expected_submatrix - actual_submatrix;
      // diff.prune(0.5, 1.0);
      // Eigen::IOFormat CleanFmt = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");
      // std::cout << submatrix.first << " diff = \n"
      //           << diff.toDense().format(CleanFmt) << std::endl;
      // std::cout << submatrix.first << " expected = \n"
      //           << expected_submatrix.toDense().format(CleanFmt) <<
      //           std::endl;
      // std::cout << submatrix.first << " actual = \n"
      //           << actual_submatrix.toDense().format(CleanFmt) << std::endl;
      // //* Debugging
    }
  }

  return error_msg;
}

Problem getProblem(std::string data_subdir) {
  std::string pyfg_path = getTestDataFpath(data_subdir, "factor_graph.pyfg");
  Problem problem = parsePyfgTextToProblem(pyfg_path);
  return problem;
}

Matrix getRandInit(std::string data_subdir) {
  std::string init_path = getTestDataFpath(data_subdir, "X_rand_dim2.mm");
  Matrix x0 = readMatrixMarketFile(init_path).toDense();
  return x0;
}

Matrix getGroundTruthState(std::string data_subdir) {
  std::string gt_path = getTestDataFpath(data_subdir, "X_gt.mm");
  Matrix X_gt = readMatrixMarketFile(gt_path).toDense();
  return X_gt;
}

Matrix getRandDX(std::string data_subdir) {
  std::string rand_dx_path = getTestDataFpath(data_subdir, "rand_dX.mm");
  Matrix rand_dx = readMatrixMarketFile(rand_dx_path).toDense();
  return rand_dx;
}

Scalar getExpectedCost(std::string data_subdir) {
  Scalar expected_cost;
  if (data_subdir == "small_ra_slam_problem") {
    expected_cost = 1.063888372855624e+03;
  } else if (data_subdir == "single_rpm") {
    expected_cost = 0.809173848024762;
  } else if (data_subdir == "single_range") {
    expected_cost = 4.718031199983851;
  } else {
    throw std::runtime_error("Do not have expected cost for: " + data_subdir);
  }
  return expected_cost;
}

Matrix getExpectedEgrad(std::string data_subdir) {
  std::string egrad_path = getTestDataFpath(data_subdir, "expected_egrad.mm");
  Matrix Egrad = readMatrixMarketFile(egrad_path).toDense();
  return Egrad;
}

Matrix getExpectedRgrad(std::string data_subdir) {
  std::string rgrad_path = getTestDataFpath(data_subdir, "expected_rgrad.mm");
  Matrix Rgrad = readMatrixMarketFile(rgrad_path).toDense();
  return Rgrad;
}

Matrix getExpectedHessProd(std::string data_subdir) {
  std::string hess_prod_path = getTestDataFpath(data_subdir, "hessProd.mm");
  Matrix hess_prod = readMatrixMarketFile(hess_prod_path).toDense();
  return hess_prod;
}
} // namespace CORA
