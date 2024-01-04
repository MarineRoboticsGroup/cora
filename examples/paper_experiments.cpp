#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/CORA_utils.h>
#include <CORA/Symbol.h>
#include <CORA/pyfg_text_parser.h>

#include <filesystem>
#include <set>
#include <vector>

std::vector<std::vector<CORA::Symbol>>
getRobotPoseChains(const CORA::Problem &problem) {
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
  std::vector<std::vector<CORA::Symbol>> robot_pose_chains;
  for (auto const &pose_char : unique_pose_chars) {
    std::vector<CORA::Symbol> robot_pose_chain =
        problem.getPoseSymbols(pose_char);
    std::sort(robot_pose_chain.begin(), robot_pose_chain.end());
    robot_pose_chains.push_back(robot_pose_chain);
  }

  // return the robot pose chains
  return robot_pose_chains;
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
  std::vector<std::vector<CORA::Symbol>> robot_pose_chains =
      getRobotPoseChains(problem);

  // if tiers.pyfg, then we have four robots
  if (pyfg_fpath == "data/tiers.pyfg" && robot_pose_chains.size() != 4) {
    throw std::runtime_error("Expected 4 robots in tiers.pyfg");
  }

  // enumerate over the robot pose chains
  for (size_t robot_index = 0; robot_index < robot_pose_chains.size();
       robot_index++) {
    // get the robot pose chain
    std::vector<CORA::Symbol> robot_pose_chain = robot_pose_chains[robot_index];

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

  CORA::Matrix x0 = problem.getRandomInitialGuess();
  int max_rank = 10;

  // start timer
  auto start = std::chrono::high_resolution_clock::now();

  // solve the problem
  CORA::CoraResult soln = CORA::solveCORA(problem, x0, max_rank);

  // end timer
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "CORA took " << elapsed.count() << " seconds" << std::endl;

  CORA::Matrix aligned_soln = problem.alignEstimateToOrigin(soln.first.x);
  saveSolutions(problem, aligned_soln, pyfg_fpath);

  return aligned_soln;
}

int main(int argc, char **argv) {
  std::vector<std::string> files = {
      "data/marine_two_robots.pyfg", "data/plaza1.pyfg", "data/plaza2.pyfg",
      "data/single_drone.pyfg", "data/tiers.pyfg"};

  for (auto file : files) {
    CORA::Matrix soln = solveProblem(file);
    std::cout << std::endl;
  }
}
