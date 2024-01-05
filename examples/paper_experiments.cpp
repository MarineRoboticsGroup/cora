#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/Symbol.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <set>
#include <vector>

using PoseChain = std::vector<CORA::Symbol>;
using PoseChains = std::vector<PoseChain>;
using RPM = CORA::RelativePoseMeasurement;

PoseChains getRobotPoseChains(const CORA::Problem &problem) {
  // get all of the unique pose characters
  std::set<unsigned char> seen_pose_chars;
  for (auto const &all_pose_symbols : problem.getPoseSymbolMap()) {
    CORA::Symbol pose_symbol = all_pose_symbols.first;
    seen_pose_chars.insert(pose_symbol.chr());
  }

  // get a sorted list of the unique pose characters
  std::vector<unsigned char> unique_pose_chars = {seen_pose_chars.begin(),
                                                  seen_pose_chars.end()};
  std::sort(unique_pose_chars.begin(), unique_pose_chars.end());

  // for each unique pose character, get the pose symbols (sorted)
  PoseChains robot_pose_chains;
  for (auto const &pose_char : unique_pose_chars) {
    PoseChain robot_pose_chain = problem.getPoseSymbols(pose_char);
    std::sort(robot_pose_chain.begin(), robot_pose_chain.end());
    robot_pose_chains.push_back(robot_pose_chain);
  }

  // return the robot pose chains
  return robot_pose_chains;
}

CORA::Matrix getRandomStartPose(const int dim) {
  CORA::Matrix rot_rand = CORA::Matrix::Random(dim, dim);

  // make sure that the rotation matrix is orthogonal by taking the QR
  // decomposition
  Eigen::HouseholderQR<CORA::Matrix> qr(rot_rand);
  CORA::Matrix rot = qr.householderQ();

  // if the determinant is negative, multiply by an identity matrix
  // with the last element negated
  if (rot.determinant() < 0) {
    CORA::Matrix neg_identity = CORA::Matrix::Identity(dim, dim);
    neg_identity(dim - 1, dim - 1) = -1;
    rot = rot * neg_identity;
  }

  // get a random translation
  CORA::Matrix tran = CORA::Matrix::Random(dim, 1) * 10;

  CORA::Matrix start_pose = CORA::Matrix::Identity(dim + 1, dim + 1);
  start_pose.block(0, 0, dim, dim) = rot;
  start_pose.block(0, dim, dim, 1) = tran;

  return start_pose;
}

std::vector<std::vector<RPM>> getOdomChains(const CORA::Problem &problem) {
  // get the relevant problem data
  PoseChains pose_chains = getRobotPoseChains(problem);
  std::vector<unsigned char> pose_chain_chars = {};
  for (auto const &pose_chain : pose_chains) {
    pose_chain_chars.push_back(pose_chain[0].chr());
  }

  // init the odom chains
  std::vector<std::vector<RPM>> odom_chains = {};
  for (auto const &pose_chain : pose_chains) {
    std::vector<RPM> odom_chain = {};
    odom_chains.push_back(odom_chain);
  }

  // iterate over the RPMs and add any odometry measurements to
  // the corresponding odom chain
  for (const RPM &measure : problem.getRPMs()) {
    // is odom if the first and second pose symbols have the same starting
    // character and adjacent indices
    if (measure.first_id.chr() == measure.second_id.chr() &&
        measure.first_id.index() + 1 == measure.second_id.index()) {
      // get the pose chain index
      auto it = std::find(pose_chain_chars.begin(), pose_chain_chars.end(),
                          measure.first_id.chr());
      size_t pose_chain_index = std::distance(pose_chain_chars.begin(), it);

      // get the odom chain index
      size_t odom_chain_index = measure.first_id.index();

      // add the odom measurement to the odom chain
      odom_chains[pose_chain_index].push_back(measure);
    }
  }

  // make sure that the odom chains are of the correct size (pose chain size -
  // 1)
  for (size_t pose_chain_index = 0; pose_chain_index < pose_chains.size();
       pose_chain_index++) {
    PoseChain pose_chain = pose_chains[pose_chain_index];
    std::vector<RPM> odom_chain = odom_chains[pose_chain_index];

    if (odom_chain.size() != pose_chain.size() - 1) {
      throw std::runtime_error(
          "Expected odom chain size to be pose chain size - 1");
      // "Expected odom chain size to be pose chain size " +
      // "- 1. The pose chain size is " + std::to_string(pose_chain.size()) +
      // " and the odom chain size is " + std::to_string(odom_chain.size()));
    }
  }

  // sort the odom chains by the first pose symbol index
  for (size_t pose_chain_index = 0; pose_chain_index < pose_chains.size();
       pose_chain_index++) {
    PoseChain pose_chain = pose_chains[pose_chain_index];
    std::vector<RPM> odom_chain = odom_chains[pose_chain_index];

    std::sort(odom_chain.begin(), odom_chain.end(),
              [](const RPM &a, const RPM &b) {
                return a.first_id.index() < b.first_id.index();
              });
  }

  // return the odom chains
  return odom_chains;
}

CORA::Matrix getOdomInitialization(const CORA::Problem &problem) {
  CORA::Matrix x0 = CORA::Matrix::Zero(problem.getDataMatrixSize(),
                                       problem.getRelaxationRank());
  // x0 = problem.getRandomInitialGuess();

  int dim = problem.dim();
  /** SET THE POSE VARIABLES  **/

  // iterate over the odom chains
  for (const std::vector<RPM> &odom_chain : getOdomChains(problem)) {
    CORA::Matrix cur_pose = getRandomStartPose(dim);
    Index cur_rot_start = problem.getRotationIdx(odom_chain[0].first_id) * dim;
    Index cur_tran_start = problem.getTranslationIdx(odom_chain[0].first_id);

    // set the first rotation and translation
    x0.block(cur_rot_start, 0, dim, dim) =
        cur_pose.block(0, 0, dim, dim).transpose();
    x0.block(cur_tran_start, 0, 1, dim) =
        cur_pose.block(0, dim, dim, 1).transpose();

    // iterate over the odometry measurements
    for (const RPM &measure : odom_chain) {
      // update the current pose
      CORA::Matrix measure_as_matrix = measure.getHomogeneousMatrix();
      cur_pose = cur_pose * measure_as_matrix;

      // update the indices
      cur_rot_start = problem.getRotationIdx(measure.second_id) * dim;
      cur_tran_start = problem.getTranslationIdx(measure.second_id);

      // set the rotation and translation
      x0.block(cur_rot_start, 0, dim, dim) =
          cur_pose.block(0, 0, dim, dim).transpose();
      x0.block(cur_tran_start, 0, 1, dim) =
          cur_pose.block(0, dim, dim, 1).transpose();
    }
  }

  /** SET THE SPHERE VARIABLES **/
  for (const auto &measure : problem.getRangeMeasurements()) {
    CORA::SymbolPair pair = measure.getSymbolPair();
    Index range_start_idx = problem.getRangeIdx(pair);
    Index first_trans_idx = problem.getTranslationIdx(pair.first);
    Index second_trans_idx = problem.getTranslationIdx(pair.second);
    x0.row(range_start_idx) =
        x0.row(second_trans_idx) - x0.row(first_trans_idx);

    // if the row is near zero, set it to a random unit vector
    // otherwise, normalize it
    if (x0.row(range_start_idx).norm() < 1e-6) {
      x0.row(range_start_idx) = CORA::Matrix::Random(1, dim);
      x0.row(range_start_idx) =
          x0.row(range_start_idx) / x0.row(range_start_idx).norm();
    } else {
      x0.row(range_start_idx) =
          x0.row(range_start_idx) / x0.row(range_start_idx).norm();
    }
  }

  // rotate the solution so that it is generically dense
  CORA::Matrix rot = CORA::Matrix::Random(problem.getRelaxationRank(),
                                          problem.getRelaxationRank());
  // orthogonalize the rotation matrix
  rot = rot.householderQr().householderQ();

  // if the determinant is negative, then multiply by identity
  // matrix with -1 in the last row
  if (rot.determinant() < 0) {
    CORA::Matrix reflector = CORA::Matrix::Identity(
        problem.getRelaxationRank(), problem.getRelaxationRank());
    reflector(problem.getRelaxationRank() - 1,
              problem.getRelaxationRank() - 1) = -1;
    rot = rot * reflector;
  }

  // the determinant should be 1
  if (std::abs(rot.determinant() - 1) > 1e-6) {
    throw std::runtime_error("Expected determinant to be 1");
  }

  // rotate the solution
  x0 = x0 * rot;

  // add small noise to the solution
  x0 = x0 + 1e-8 * CORA::Matrix::Random(x0.rows(), x0.cols());

  return x0;
}

void saveSolutions(const CORA::Problem &problem,
                   const CORA::Matrix &aligned_soln,
                   const std::string &pyfg_fpath) {
  /**
   * @brief for each robot, save the solution to a .tum file e.g.
   * data/plaza2.pyfg -> /tmp/plaza2/cora_0.tum
   * or
   * data/marine_two_robots.pyfg -> /tmp/marine_two_robots/cora_0.tum,
   * /tmp/marine_two_robots/cora_1.tum
   *
   */

  // strip the .pyfg extension and the data/ prefix
  size_t data_length = std::string("data/").length();
  size_t pyfg_index = pyfg_fpath.find(".pyfg");
  std::string save_dir_name =
      pyfg_fpath.substr(data_length, pyfg_index - data_length);
  std::string save_dir_path = "/tmp/" + save_dir_name;

  // create the directory if it does not exist
  if (!std::filesystem::exists(save_dir_path)) {
    std::filesystem::create_directory(save_dir_path);
  }

  std::string save_path = save_dir_path + "/cora_";

  // get the different robot pose chains
  PoseChains robot_pose_chains = getRobotPoseChains(problem);

  // if tiers.pyfg, then we have four robots
  if (pyfg_fpath == "data/tiers.pyfg" && robot_pose_chains.size() != 4) {
    throw std::runtime_error("Expected 4 robots in tiers.pyfg");
  }

  // enumerate over the robot pose chains
  for (size_t robot_index = 0; robot_index < robot_pose_chains.size();
       robot_index++) {
    // get the robot pose chain
    PoseChain robot_pose_chain = robot_pose_chains[robot_index];

    // save the estimated poses for this robot
    std::string robot_save_path =
        save_path + std::to_string(robot_index) + ".tum";
    saveSolnToTum(robot_pose_chain, problem, aligned_soln, robot_save_path);
  }
}

CORA::Matrix solveProblem(std::string pyfg_fpath) {
  std::cout << "Solving " << pyfg_fpath << std::endl;

  CORA::Problem problem = CORA::parsePyfgTextToProblem("./bin/" + pyfg_fpath);
  problem.updateProblemData();

  // CORA::Matrix x0 = problem.getRandomInitialGuess();
  CORA::Matrix x0 = getOdomInitialization(problem);
  int max_rank = 10;

  // start timer
  auto start = std::chrono::high_resolution_clock::now();

  // solve the problem
  bool verbose = false;
  CORA::CoraResult soln = CORA::solveCORA(problem, x0, max_rank, verbose);

  // end timer
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "CORA took " << elapsed.count() << " seconds" << std::endl;

  CORA::Matrix aligned_soln = problem.alignEstimateToOrigin(soln.first.x);
  saveSolutions(problem, aligned_soln, pyfg_fpath);

  return aligned_soln;
}

int main(int argc, char **argv) {
  std::vector<std::string> files = {// "data/marine_two_robots.pyfg",
                                    "data/plaza1.pyfg", "data/plaza2.pyfg",
                                    "data/single_drone.pyfg",
                                    "data/tiers.pyfg"};

  for (auto file : files) {
    CORA::Matrix soln = solveProblem(file);
    std::cout << std::endl;
  }
}
