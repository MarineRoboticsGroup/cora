
#include <CORA/CORA_utils.h>

#include <unsupported/Eigen/SparseExtra>

#include <filesystem>
#include <fstream>
#include <iostream>

CORA::SparseMatrix CORA::readMatrixMarketFile(const std::string &filename) {
  CORA::SparseMatrix A;
  bool is_symmetric = false;
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

void CORA::writeMatrixMarketFile(const SparseMatrix &A,
                                 const std::string &filename) {
  Eigen::saveMarket(A, filename);
}

void CORA::printMatrixSparsityPattern(const Matrix &matrix) {
  // if entry is zero- print "-", otherwise print "X"
  // make sure that vertical spacing is the same as horizontal spacing
  std::cout << "Matrix sparsity pattern: " << std::endl;

  int rows = matrix.rows();
  int cols = matrix.cols();

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

std::string CORA::getTestDataFpath(std::string data_subdir, std::string fname) {
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

std::string CORA::checkSubmatricesAreCorrect(CORA::Problem prob,
                                             std::string data_subdir) {
  CORA::CoraDataSubmatrices data_submatrices = prob.getDataSubmatrices();

  std::string error_msg = "";

  // map from file name to the submatrix
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
