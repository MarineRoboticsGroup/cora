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

#ifdef GPERFTOOLS
#include <gperftools/profiler.h>
#endif

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

CORA::Matrix getMostSimilarPoints(const std::vector<CORA::Matrix> &pts1,
                                  const std::vector<CORA::Matrix> &pts2,
                                  const std::vector<CORA::Matrix> &pts3) {
  CORA::Matrix nearest_pt;
  double dist = std::numeric_limits<double>::max();
  std::vector<std::pair<CORA::Matrix, CORA::Matrix>> pairs = {};
  for (const auto &pt1 : pts1) {
    for (const auto &pt2 : pts2) {
      for (const auto &pt3 : pts3) {
        pairs.push_back({pt1, pt2});
        pairs.push_back({pt1, pt3});
        pairs.push_back({pt2, pt3});
      }
    }
  }

  // iterate over pairs and find the pair with the smallest distance
  for (const auto &pair : pairs) {
    auto curr_dist = (pair.first - pair.second).norm();
    if (curr_dist < dist) {
      nearest_pt = (pair.first + pair.second) / 2;
      dist = curr_dist;
    }
  }

  std::cout << "Nearest point: " << nearest_pt << " with dist: " << dist
            << std::endl;

  return nearest_pt;
}

std::vector<CORA::Matrix> getCircleIntersect(CORA::Scalar x1, CORA::Scalar y1,
                                             CORA::Scalar r1, CORA::Scalar x2,
                                             CORA::Scalar y2, CORA::Scalar r2) {
  // get the distance between the two points
  CORA::Scalar d = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

  // check if the circles are too far apart, if so, just return the average of
  // the centers
  if (d > r1 + r2) {
    CORA::Matrix pt1(1, 2);
    pt1 << (x1 + x2) / 2, (y1 + y2) / 2;
    return {pt1};
  }

  // check if the circles are too close together
  if (d < std::abs(r1 - r2)) {
    throw std::runtime_error("Circles are too close together");
  }

  // check if the circles are the same
  if (d == 0 && r1 == r2) {
    throw std::runtime_error("Circles are the same");
  }

  // compute the distance from the first point to the intersection point
  CORA::Scalar a =
      (std::pow(r1, 2) - std::pow(r2, 2) + std::pow(d, 2)) / (2 * d);

  // compute the height of the intersection point
  CORA::Scalar h = std::sqrt(std::pow(r1, 2) - std::pow(a, 2));

  // compute the intersection points
  CORA::Scalar x3 = x1 + a * (x2 - x1) / d;
  CORA::Scalar y3 = y1 + a * (y2 - y1) / d;

  CORA::Scalar x4 = x3 + h * (y2 - y1) / d;
  CORA::Scalar y4 = y3 - h * (x2 - x1) / d;

  CORA::Scalar x5 = x3 - h * (y2 - y1) / d;
  CORA::Scalar y5 = y3 + h * (x2 - x1) / d;

  // return the intersection points
  CORA::Matrix pt1(1, 2);
  pt1 << x4, y4;
  CORA::Matrix pt2(1, 2);
  pt2 << x5, y5;
  return {pt1, pt2};
}

CORA::Matrix init2dLandmark(const CORA::Symbol &landmark_symbol,
                            const CORA::Matrix &x0,
                            const CORA::Problem &problem) {
  // get all of the range measurements
  auto range_measurements = problem.getRangeMeasurements();

  // erase any range measurements that do not involve the landmark
  range_measurements.erase(
      std::remove_if(range_measurements.begin(), range_measurements.end(),
                     [&landmark_symbol](const CORA::RangeMeasurement &measure) {
                       return measure.first_id != landmark_symbol &&
                              measure.second_id != landmark_symbol;
                     }),
      range_measurements.end());

  // we will take the translations from:
  // 1. the first range measurement
  // 2. the translation furthest from the first range measurement
  // 3. a random translation that is at least 25% of the distance between the
  // first and second translations
  CORA::Matrix first_trans, far_trans, rand_trans;

  // get the first range measurement
  CORA::RangeMeasurement first_measure = range_measurements[0];
  CORA::RangeMeasurement far_measure = range_measurements[0];
  CORA::RangeMeasurement rand_measure = range_measurements[0];
  first_trans = x0.row(problem.getTranslationIdx(first_measure.first_id));
  range_measurements.erase(range_measurements.begin());

  // get the translation furthest from the first translation
  double max_dist = 0;
  for (const auto &measure : range_measurements) {
    auto curr_trans = x0.row(problem.getTranslationIdx(measure.first_id));
    double dist = (curr_trans - first_trans).norm();
    if (dist > max_dist) {
      far_trans = curr_trans;
      far_measure = measure;
      max_dist = dist;
    }
  }

  // get a random translation that is at least 25% of the distance between the
  // first and second translations
  double rand_trans_threshold = max_dist * 0.25;
  bool found_rand_trans = false;
  for (const auto &measure : range_measurements) {
    auto curr_trans = x0.row(problem.getTranslationIdx(measure.first_id));
    double first_dist = (curr_trans - first_trans).norm();
    double far_dist = (curr_trans - far_trans).norm();
    if (first_dist > rand_trans_threshold && far_dist > rand_trans_threshold) {
      rand_trans = curr_trans;
      rand_measure = measure;
      found_rand_trans = true;
      break;
    }
  }

  // if we did not find a random translation, then just pick a random one
  if (!found_rand_trans) {
    int rand_idx = rand() % range_measurements.size();
    rand_measure = range_measurements[rand_idx];
    rand_trans = x0.row(problem.getTranslationIdx(rand_measure.first_id));
  }

  // there are circles defined by each translation/range measurement pair.
  // the translation defines the center and measure.r defines the radius.
  // we will find the intersection of these circles to get the landmark
  // position
  auto ft_x = first_trans(0, 0);
  auto ft_y = first_trans(0, 1);
  auto first_far_intersect =
      getCircleIntersect(first_trans(0, 0), first_trans(0, 1), first_measure.r,
                         far_trans(0, 0), far_trans(0, 1), far_measure.r);
  auto first_rand_intersect =
      getCircleIntersect(first_trans(0, 0), first_trans(0, 1), first_measure.r,
                         rand_trans(0, 0), rand_trans(0, 1), rand_measure.r);
  auto far_rand_intersect =
      getCircleIntersect(far_trans(0, 0), far_trans(0, 1), far_measure.r,
                         rand_trans(0, 0), rand_trans(0, 1), rand_measure.r);

  // iterate over all the different pairs between the three intersections
  // and pick the two points that are closest together
  auto best_pt = getMostSimilarPoints(first_far_intersect, first_rand_intersect,
                                      far_rand_intersect);

  std::cout << "Initializing landmark " << landmark_symbol << " to " << best_pt
            << std::endl;
  return best_pt;
}

CORA::Matrix getGtLandmarkPosition(const CORA::Symbol &landmark_symbol,
                                   const CORA::Problem &problem,
                                   std::string pyfg_fpath) {
  // check that the symbol char is 'l' or 'L'
  if (landmark_symbol.chr() != 'l' && landmark_symbol.chr() != 'L') {
    throw std::runtime_error("Expected landmark symbol to be 'l' or 'L'");
  }
  auto landmark_symbol_index = landmark_symbol.index();
  bool is_plaza1 = pyfg_fpath.find("plaza1") != std::string::npos;
  bool is_plaza2 = pyfg_fpath.find("plaza2") != std::string::npos;
  bool is_single_drone = pyfg_fpath.find("single_drone") != std::string::npos;
  bool is_tiers = pyfg_fpath.find("tiers") != std::string::npos;

  // at least one of the above should be true
  if (!(is_plaza1 || is_plaza2 || is_single_drone || is_tiers)) {
    throw std::runtime_error("Expected pyfg_fpath to be plaza1, plaza2, "
                             "single_drone, or tiers");
  }

  // no more than one of the above should be true
  auto sum_of_bools = is_plaza1 + is_plaza2 + is_single_drone + is_tiers;
  if (sum_of_bools > 1) {
    throw std::runtime_error("Expected pyfg_fpath to be plaza1, plaza2, "
                             "single_drone, or tiers");
  }

  // check if "plaza1" is in the pyfg_fpath
  CORA::Matrix gt_landmark_pos = CORA::Matrix::Zero(1, problem.dim());
  if (is_plaza1) {
    /**
     * Plaza 1:
     * VERTEX_XY L0 -46.6232339999406 11.0255489991978
     * VERTEX_XY L1 11.0361239999766 -6.95868900045753
     * VERTEX_XY L2 22.0531290000072 23.8484819997102
     * VERTEX_XY L3 -17.664892999921 59.009180999361
     */
    // switch on the landmark symbol index
    switch (landmark_symbol_index) {
    case 0:
      gt_landmark_pos(0, 0) = -46.6232339999406;
      gt_landmark_pos(0, 1) = 11.0255489991978;
      return gt_landmark_pos;
    case 1:
      gt_landmark_pos(0, 0) = 11.0361239999766;
      gt_landmark_pos(0, 1) = -6.95868900045753;
      return gt_landmark_pos;
    case 2:
      gt_landmark_pos(0, 0) = 22.0531290000072;
      gt_landmark_pos(0, 1) = 23.8484819997102;
      return gt_landmark_pos;
    case 3:
      gt_landmark_pos(0, 0) = -17.664892999921;
      gt_landmark_pos(0, 1) = 59.009180999361;
      return gt_landmark_pos;
    }
  } else if (is_plaza2) {
    /**
     *
     * Plaza 2:
     * VERTEX_XY L0 -68.9265369999921 18.3777969991788
     * VERTEX_XY L1 -37.5805369999725 69.2277969997376
     * VERTEX_XY L2 -33.6205370000098 26.9677969999611
     * VERTEX_XY L3 1.70946300006472 -5.81220300029963
     */
    switch (landmark_symbol_index) {
    case 0:
      gt_landmark_pos(0, 0) = -68.9265369999921;
      gt_landmark_pos(0, 1) = 18.3777969991788;
      return gt_landmark_pos;
    case 1:
      gt_landmark_pos(0, 0) = -37.5805369999725;
      gt_landmark_pos(0, 1) = 69.2277969997376;
      return gt_landmark_pos;
    case 2:
      gt_landmark_pos(0, 0) = -33.6205370000098;
      gt_landmark_pos(0, 1) = 26.9677969999611;
      return gt_landmark_pos;
    case 3:
      gt_landmark_pos(0, 0) = 1.70946300006472;
      gt_landmark_pos(0, 1) = -5.81220300029963;
      return gt_landmark_pos;
    }
  } else if (is_single_drone) {
    /**
     *
     * Single drone:
     * VERTEX_XYZ L0 -1.56660422992355 -3.7591615842206183 0.809476152530972
     *
     * Tiers:
     * VERTEX_XY L0 -0.45365739942811395 1.0412147360554351
     */
    switch (landmark_symbol_index) {
    case 0:
      gt_landmark_pos(0, 0) = -1.56660422992355;
      gt_landmark_pos(0, 1) = -3.7591615842206183;
      return gt_landmark_pos;
    }
  } else if (is_tiers) {
    switch (landmark_symbol_index) {
    case 0:
      gt_landmark_pos(0, 0) = -0.45365739942811395;
      gt_landmark_pos(0, 1) = 1.0412147360554351;
      return gt_landmark_pos;
    }
  }

  throw std::runtime_error("Invalid landmark symbol index");
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

CORA::Matrix getOdomInitialization(const CORA::Problem &problem,
                                   std::string pyfg_path) {
  CORA::Matrix x0 = CORA::Matrix::Zero(problem.getDataMatrixSize(),
                                       problem.getRelaxationRank());
  // x0 = problem.getRandomInitialGuess();

  int dim = problem.dim();
  /** SET THE POSE VARIABLES  **/

  // iterate over the odom chains
  bool first = true;
  for (const std::vector<RPM> &odom_chain : getOdomChains(problem)) {
    CORA::Matrix cur_pose;
    if (first) {
      cur_pose = CORA::Matrix::Identity(dim + 1, dim + 1);
      first = false;
    } else {
      cur_pose = getRandomStartPose(dim);
    }
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

  /** RANDOMIZE THE LANDMARK VARIABLES **/
  for (const auto &landmark_pair : problem.getLandmarkSymbolMap()) {
    CORA::Symbol symbol = landmark_pair.first;
    Index landmark_start_idx = problem.getTranslationIdx(symbol);
    // x0.row(landmark_start_idx) = CORA::Matrix::Random(1, x0.cols());
    x0.block(landmark_start_idx, 0, 1, dim) =
        init2dLandmark(symbol, x0, problem);
    // getGtLandmarkPosition(symbol, problem, pyfg_path);
  }

  /** SET THE SPHERE VARIABLES **/
  for (const auto &measure : problem.getRangeMeasurements()) {
    CORA::SymbolPair pair = measure.getSymbolPair();
    Index range_start_idx = problem.getRangeIdx(pair);
    Index first_trans_idx = problem.getTranslationIdx(pair.first);
    Index second_trans_idx = problem.getTranslationIdx(pair.second);

    CORA::Matrix diff = x0.row(second_trans_idx) - x0.row(first_trans_idx);
    x0.row(range_start_idx) = diff / diff.norm();

    bool dist_very_off =
        diff.norm() / measure.r < 0.1 || diff.norm() / measure.r > 10;
    bool second_idx_is_landmark =
        pair.second.chr() == 'l' || pair.second.chr() == 'L';
    if (dist_very_off && second_idx_is_landmark) {
      // std::cout << "Range measure indicates that the landmark is too close
      // or
      // "
      //              "too far apart! Measured dist: "
      //           << measure.r << "Initialized dist: " << diff.norm()
      //           << std::endl;
    }

    //   // pick a random unit vector and offset the landmark in that
    //   direction
    //   // CORA::Matrix random_unit_vector = CORA::Matrix::Random(1,
    //   x0.cols());
    //   // random_unit_vector = random_unit_vector /
    //   random_unit_vector.norm();
    //   // x0.row(range_start_idx) = random_unit_vector;
    //   // x0.row(second_trans_idx) =
    //   //     x0.row(first_trans_idx) + random_unit_vector * measure.r;
    // } else {
    // }
    // x0.row(range_start_idx) = diff / diff.norm();

    // if the row is near zero, set it to a random unit vector
    // otherwise, normalize it
    if (false && x0.row(range_start_idx).norm() < 1e-5) {
      std::cout << "Setting range to random unit vector! Measured dist: "
                << measure.r << std::endl;
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
  // x0 = x0 + 1e-8 * CORA::Matrix::Random(x0.rows(), x0.cols());

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
  CORA::Matrix x0 = getOdomInitialization(problem, pyfg_fpath);
  int max_rank = 10;

#ifdef GPERFTOOLS
  ProfilerStart("cora.prof");
#endif

  // start timer
  auto start = std::chrono::high_resolution_clock::now();

  // solve the problem
  bool verbose = true;
  bool log_iterates = false;
  CORA::CoraResult soln =
      CORA::solveCORA(problem, x0, max_rank, verbose, log_iterates);

  // end timer
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "CORA took " << elapsed.count() << " seconds" << std::endl;

#ifdef GPERFTOOLS
  ProfilerStop();
#endif

  CORA::Matrix aligned_soln = problem.alignEstimateToOrigin(soln.first.x);
  saveSolutions(problem, aligned_soln, pyfg_fpath);

  return aligned_soln;
}

int main(int argc, char **argv) {
  std::vector<std::string> files = {// "data/marine_two_robots.pyfg",
                                    // "data/plaza1.pyfg", "data/plaza2.pyfg",
                                    // "data/single_drone.pyfg",
                                    "data/tiers.pyfg"};

  for (auto file : files) {
    CORA::Matrix soln = solveProblem(file);
    std::cout << std::endl;
  }
}
