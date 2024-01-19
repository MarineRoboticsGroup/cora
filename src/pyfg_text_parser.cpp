//
// Created by Tim Magoun on 10/31/23.
//
#include <CORA/pyfg_text_parser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <set>

namespace CORA {

Matrix fromAngle(double angle_rad);
Matrix fromQuat(double qx, double qy, double qz, double qw);

Vector readVector(std::istringstream &iss, int dim);
Scalar readScalar(std::istringstream &iss);
Matrix readQuat(std::istringstream &iss);
Matrix readSymmetric(std::istringstream &iss, int dim);

/**
 * @brief Enum to keep track of the different types of items in a PyFG file
 *
 * @details This contains all of the types in the PyFG format that we currently
 * have support for.
 */
enum PyFGType {
  POSE_TYPE_2D,
  POSE_TYPE_3D,
  POSE_PRIOR_2D,
  POSE_PRIOR_3D,
  LANDMARK_TYPE_2D,
  LANDMARK_TYPE_3D,
  LANDMARK_PRIOR_2D,
  LANDMARK_PRIOR_3D,
  REL_POSE_POSE_TYPE_2D,
  REL_POSE_POSE_TYPE_3D,
  REL_POSE_LANDMARK_TYPE_2D,
  REL_POSE_LANDMARK_TYPE_3D,
  RANGE_MEASURE_TYPE,
};

int getDimFromPyfgFirstLine(const std::string &filename) {
  // Check if the file exists and we can read it
  std::ifstream in_file(filename);
  if (!in_file.good()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  const std::map<std::string, PyFGType> PyFGStringToType{
      {"VERTEX_SE2", POSE_TYPE_2D},
      {"VERTEX_SE3:QUAT", POSE_TYPE_3D},
      {"VERTEX_SE2:PRIOR", POSE_PRIOR_2D},
      {"VERTEX_SE3:QUAT:PRIOR", POSE_PRIOR_3D},
      {"VERTEX_XY", LANDMARK_TYPE_2D},
      {"VERTEX_XYZ", LANDMARK_TYPE_3D},
      {"VERTEX_XY:PRIOR", LANDMARK_PRIOR_2D},
      {"VERTEX_XYZ:PRIOR", LANDMARK_PRIOR_3D},
      {"EDGE_SE2", REL_POSE_POSE_TYPE_2D},
      {"EDGE_SE3:QUAT", REL_POSE_POSE_TYPE_3D},
      {"EDGE_SE2_XY", REL_POSE_LANDMARK_TYPE_2D},
      {"EDGE_SE3_XYZ", REL_POSE_LANDMARK_TYPE_3D},
      {"EDGE_RANGE", RANGE_MEASURE_TYPE}};

  // get just the first line and close the file
  std::string line;
  std::getline(in_file, line);
  in_file.close();

  // Get the item type with the first word
  std::istringstream iss(line);
  std::string item_type;

  double timestamp;
  // A bunch of placeholder strings to be used for populating different types
  std::string sym1, sym2;

  if (!(iss >> item_type)) {
    throw std::runtime_error("Could not read item type from line " + line);
  }

  if (PyFGStringToType.find(item_type) == PyFGStringToType.end()) {
    throw std::runtime_error("Unknown item type " + item_type);
  }

  switch (PyFGStringToType.find(item_type)->second) {
  case POSE_TYPE_2D:
    return 2;
  case POSE_TYPE_3D:
    return 3;
  case LANDMARK_TYPE_2D:
    return 2;
  case LANDMARK_TYPE_3D:
    return 3;
  default:
    throw std::runtime_error("Could not determine dimension from first line " +
                             line);
  }
}

/**
 * @brief Parses a text file written in the PyFG format and returns a
 * CORA::Problem
 *
 * @details This function reads line by line, calling the CORA::Problem
 * constructor. It will throw an exception if the input is not formatted
 * correctly or it fails to read a number or string that is expected. It assumes
 * that the symbols in PyFG are composed of a single character and a number,
 * e.g. (A0). It does not do any validation aside from the proper formatting of
 * the PyFG file.
 * @param filename Path to the PyFG file
 * @return CORA::Problem The parsed problem
 */
Problem parsePyfgTextToProblem(const std::string &filename) {
  // Note: This currently ignores all groundtruth measurements embedded
  // in the file
  int dim = getDimFromPyfgFirstLine(filename);
  int relaxation_rank = dim;
  CORA::Formulation formulation = CORA::Formulation::Explicit;
  CORA::Preconditioner preconditioner =
      CORA::Preconditioner::RegularizedCholesky;
  CORA::Problem problem(dim, relaxation_rank, formulation, preconditioner);

  const std::map<std::string, PyFGType> PyFGStringToType{
      {"VERTEX_SE2", POSE_TYPE_2D},
      {"VERTEX_SE3:QUAT", POSE_TYPE_3D},
      {"VERTEX_SE2:PRIOR", POSE_PRIOR_2D},
      {"VERTEX_SE3:QUAT:PRIOR", POSE_PRIOR_3D},
      {"VERTEX_XY", LANDMARK_TYPE_2D},
      {"VERTEX_XYZ", LANDMARK_TYPE_3D},
      {"VERTEX_XY:PRIOR", LANDMARK_PRIOR_2D},
      {"VERTEX_XYZ:PRIOR", LANDMARK_PRIOR_3D},
      {"EDGE_SE2", REL_POSE_POSE_TYPE_2D},
      {"EDGE_SE3:QUAT", REL_POSE_POSE_TYPE_3D},
      {"EDGE_SE2_XY", REL_POSE_LANDMARK_TYPE_2D},
      {"EDGE_SE3_XYZ", REL_POSE_LANDMARK_TYPE_3D},
      {"EDGE_RANGE", RANGE_MEASURE_TYPE}};

  // Check if the file exists and we can read it
  std::ifstream in_file(filename);
  if (!in_file.good()) {
    throw std::runtime_error("Could not open file " + filename);
  }

  std::string line;
  while (std::getline(in_file, line)) {
    // Get the item type with the first word
    std::istringstream iss(line);
    std::string item_type;

    double timestamp;
    // A bunch of placeholder strings to be used for populating different types
    std::string sym1, sym2;

    if (!(iss >> item_type)) {
      throw std::runtime_error("Could not read item type from line " + line);
    }

    if (PyFGStringToType.find(item_type) == PyFGStringToType.end()) {
      throw std::runtime_error("Unknown item type " + item_type);
    }

    switch (PyFGStringToType.find(item_type)->second) {
    case POSE_TYPE_2D:
    case POSE_TYPE_3D:
      // VERTEX_SE3 ts sym x y z qx qy qz qw
      // VERTEX_SE2 ts sym x y theta

      // The GT pose is encoded but we ignore it
      if (iss >> timestamp >> sym1) {
        Symbol sym(sym1);
        problem.addPoseVariable(sym);
      } else {
        throw std::runtime_error("Could not read pose variable from line " +
                                 line);
      }
      break;

    case POSE_PRIOR_2D:
      if (iss >> timestamp >> sym1) {
        auto xy = readVector(iss, 2);
        auto R = fromAngle(readScalar(iss));
        auto cov = readSymmetric(iss, 3);
        Symbol sym(sym1);
        PosePrior pose_prior{sym, R, xy, cov};
        problem.addPosePrior(pose_prior);
      } else {
        throw std::runtime_error("Could not read pose prior from line " + line);
      }
      break;

    case POSE_PRIOR_3D:
      if (iss >> timestamp >> sym1) {
        auto xyz = readVector(iss, 3);
        auto R = readQuat(iss);
        auto cov = readSymmetric(iss, 6);
        Symbol sym(sym1);
        PosePrior pose_prior{sym, R, xyz, cov};
        problem.addPosePrior(pose_prior);
      } else {
        throw std::runtime_error("Could not read pose prior from line " + line);
      }
      break;

    case LANDMARK_TYPE_2D:
    case LANDMARK_TYPE_3D:
      // Note: Landmark types are the only PyFG item that doesn't have a
      // timestamp in its text format
      if (iss >> sym1) {
        Symbol sym(sym1);
        problem.addLandmarkVariable(sym);
      } else {
        throw std::runtime_error("Could not read landmark variable from line " +
                                 line);
      }
      break;

    case LANDMARK_PRIOR_2D:
      if (iss >> timestamp >> sym1) {
        auto xy = readVector(iss, 2);
        auto cov = readSymmetric(iss, 2);
        Symbol sym(sym1);
        LandmarkPrior landmark_prior{sym, xy, cov};
        problem.addLandmarkPrior(landmark_prior);
      } else {
        throw std::runtime_error("Could not read landmark prior from line " +
                                 line);
      }
      break;

    case LANDMARK_PRIOR_3D:
      if (iss >> timestamp >> sym1) {
        auto xyz = readVector(iss, 3);
        auto cov = readSymmetric(iss, 3);
        Symbol sym(sym1);
        LandmarkPrior landmark_prior{sym, xyz, cov};
        problem.addLandmarkPrior(landmark_prior);
      } else {
        throw std::runtime_error("Could not read landmark prior from line " +
                                 line);
      }
      break;

    case REL_POSE_POSE_TYPE_2D:
      if (iss >> timestamp >> sym1 >> sym2) {
        auto xy = readVector(iss, 2);
        auto R = fromAngle(readScalar(iss));
        auto cov = readSymmetric(iss, 3);
        Symbol sym_a(sym1);
        Symbol sym_b(sym2);
        RelativePoseMeasurement rel_pose{sym_a, sym_b, R, xy, cov};
        problem.addRelativePoseMeasurement(rel_pose);
      } else {
        throw std::runtime_error(
            "Could not read relative pose measurement from line " + line);
      }
      break;

    case REL_POSE_POSE_TYPE_3D:
      if (iss >> timestamp >> sym1 >> sym2) {
        auto xyz = readVector(iss, 3);
        auto R = readQuat(iss);
        auto cov = readSymmetric(iss, 6);
        Symbol sym_a(sym1);
        Symbol sym_b(sym2);
        RelativePoseMeasurement rel_pose{sym_a, sym_b, R, xyz, cov};
        problem.addRelativePoseMeasurement(rel_pose);
      } else {
        throw std::runtime_error(
            "Could not read relative pose measurement from line " + line);
      }
      break;

    case REL_POSE_LANDMARK_TYPE_2D:
      if (iss >> timestamp >> sym1 >> sym2) {
        auto xy = readVector(iss, 2);
        auto cov = readSymmetric(iss, 2);
        Symbol sym_a(sym1);
        Symbol sym_b(sym2);
        RelativePoseLandmarkMeasurement rel_pose_landmark{sym_a, sym_b, xy,
                                                          cov};
        problem.addRelativePoseLandmarkMeasurement(rel_pose_landmark);
      } else {
        throw std::runtime_error(
            "Could not read relative pose-landmark measurement from line " +
            line);
      }
      break;

    case REL_POSE_LANDMARK_TYPE_3D:
      if (iss >> timestamp >> sym1 >> sym2) {
        auto xyz = readVector(iss, 3);
        auto cov = readSymmetric(iss, 3);
        Symbol sym_a(sym1);
        Symbol sym_b(sym2);
        RelativePoseLandmarkMeasurement rel_pose_landmark{sym_a, sym_b, xyz,
                                                          cov};
        problem.addRelativePoseLandmarkMeasurement(rel_pose_landmark);
      } else {
        throw std::runtime_error(
            "Could not read relative pose-landmark measurement from line " +
            line);
      }
      break;

    case RANGE_MEASURE_TYPE:
      if (iss >> timestamp >> sym1 >> sym2) {
        auto range = readScalar(iss);
        auto cov = readScalar(iss);
        Symbol sym_a(sym1);
        Symbol sym_b(sym2);
        RangeMeasurement range_measurement{sym_a, sym_b, range, cov};
        problem.addRangeMeasurement(range_measurement);
      } else {
        throw std::runtime_error("Could not read range measurement from line " +
                                 line);
      }
      break;
    }
  }
  in_file.close();
  return problem;
}

Matrix fromAngle(double angle_rad) {
  Matrix rotation_matrix_2d(2, 2);
  rotation_matrix_2d << cos(angle_rad), -sin(angle_rad), sin(angle_rad),
      cos(angle_rad);
  return rotation_matrix_2d;
}

Matrix fromQuat(double qx, double qy, double qz, double qw) {
  Eigen::Quaterniond q(qw, qx, qy, qz);
  auto rot_mat = q.toRotationMatrix();
  // Not sure why we can't cast it directly?
  Matrix result(3, 3);
  result << rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2), rot_mat(1, 0),
      rot_mat(1, 1), rot_mat(1, 2), rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2);
  return result;
}

Scalar readScalar(std::istringstream &iss) {
  Scalar result;
  if (iss >> result) {
    return result;
  } else {
    throw std::runtime_error("Could not read scalar");
  }
}

Vector readVector(std::istringstream &iss, int dim) {
  Vector result(dim);
  for (int i{0}; i < dim; i++) {
    if (iss >> result(i)) {
      continue;
    } else {
      throw std::runtime_error("Could not read vector");
    }
  }
  return result;
}

/**
 * @brief Reads a xyzw quaternion from a string stream
 * @param iss string stream to read from
 * @return Rotation matrix representation of the quaternion
 */
Matrix readQuat(std::istringstream &iss) {
  Vector result(4, 1);
  for (int i{0}; i < 4; i++) {
    if (iss >> result(i)) {
      continue;
    } else {
      throw std::runtime_error("Could not read quaternion");
    }
  }
  return fromQuat(result(0), result(1), result(2), result(3));
}

/**
 * @brief Reads a symmetric matrix from a string stream in column-major order
 *
 * @param iss input stream to read from
 * @param dim dimension of the matrix
 * @return dim x dim matrix of doubles
 */
Matrix readSymmetric(std::istringstream &iss, int dim) {
  Matrix cov(dim, dim);
  double val;
  for (int i{0}; i < dim; i++) {
    for (int j{i}; j < dim; j++) {
      if (iss >> val) {
        cov(i, j) = val;
        cov(j, i) = val;
      } else {
        std::cout << "Attempted to parse covariance matrix. i:" << i
                  << " j:" << j << " val:" << val << std::endl;
        throw std::runtime_error("Could not read covariance matrix");
      }
    }
  }
  return cov;
}
} // namespace CORA
