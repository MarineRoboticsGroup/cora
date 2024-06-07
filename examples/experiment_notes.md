# Experiment Notes and Timing Results

## Original Experiments

| Experiment   | CORA (seconds) | GTSAM (seconds) | # Poses | # Landmarks | # Range Measurements | # RPM Measurements |
| ------------ | -------------- | --------------- | ------- | ----------- | -------------------- | ------------------ |
| Single Drone | 1.21           | 1.73            | 1754    | 1           | 1754                 | 1753               |
| Plaza 1      | 8.93           | 8.57            | 9658    | 4           | 3326                 | 9657               |
| Plaza 2      | 1.96           | 5.05            | 4091    | 4           | 1807                 | 4090               |
| TIERS        | 11.30          | 6.74            | 9768    | 1           | 7789                 | 9764               |
| MR.CLAM 2    | 28.30          | 43.53           | 31067   | 15          | 8392                 | 36465              |
| MR.CLAM 4    | 27.38          | 39.93           | 26842   | 15          | 6561                 | 32503              |
| MR.CLAM 6    | 8.81           | 7.05            | 11274   | 15          | 3537                 | 13121              |
| MR.CLAM 7    | 9.13           | 23.23           | 13720   | 15          | 4004                 | 16056              |

<!-- PYFG summaries

Dataset: /home/alan/cora/examples/data/single_drone.pyfg
[factor_graph.py:241] py_factor_graph.utils.logging_utils INFO - Robots: 1 || Variables: 1754 poses, 1 landmarks || Measurements: 1753 odom, 0 pose to landmark, 1754 range, 0 loop closures Interrobot loop closures: No inter-robot loop closures

Dataset: /home/alan/cora/examples/data/plaza1.pyfg
[factor_graph.py:241] py_factor_graph.utils.logging_utils INFO - Robots: 1 || Variables: 9658 poses, 4 landmarks || Measurements: 9657 odom, 0 pose to landmark, 3326 range, 0 loop closures Interrobot loop closures: No inter-robot loop closures

Dataset: /home/alan/cora/examples/data/plaza2.pyfg
[factor_graph.py:241] py_factor_graph.utils.logging_utils INFO - Robots: 1 || Variables: 4091 poses, 4 landmarks || Measurements: 4090 odom, 0 pose to landmark, 1807 range, 0 loop closures Interrobot loop closures: No inter-robot loop closures

Dataset: /home/alan/cora/examples/data/tiers.pyfg
[factor_graph.py:241] py_factor_graph.utils.logging_utils INFO - Robots: 4 || Variables: 9768 poses, 1 landmarks || Measurements: 9764 odom, 0 pose to landmark, 7789 range, 0 loop closures Interrobot loop closures: No inter-robot loop closures

-->



<!-- GTSAM on a single core

Problem: plaza1.pyfg, strategy: gtsam_gt_pose_random_landmarks, time: 8.57 secs
Problem: plaza2.pyfg, strategy: gtsam_gt_pose_random_landmarks, time: 5.05 secs
Problem: single_drone.pyfg, strategy: gtsam_gt_pose_random_landmarks, time: 1.73 secs
Problem: tiers.pyfg, strategy: gtsam_gt_pose_random_landmarks, time: 6.74 secs

Problem: mrclam/range_and_rpm/mrclam2/mrclam2.pyfg, strategy: gtsam_random_pose_random_landmarks, time: 43.53 secs
Problem: mrclam/range_and_rpm/mrclam4/mrclam4.pyfg, strategy: gtsam_random_pose_random_landmarks, time: 39.93 secs
Problem: mrclam/range_and_rpm/mrclam6/mrclam6.pyfg, strategy: gtsam_random_pose_random_landmarks, time: 7.05 secs
Problem: mrclam/range_and_rpm/mrclam7/mrclam7.pyfg, strategy: gtsam_random_pose_random_landmarks, time: 23.23 secs
 -->

 <!-- MATLAB preconditioners 

 the block-cholesky preconditioner requires about half as many iterations as the
 regularized cholesky preconditioner

MATLAB
    block chol: 917 + 745 + 1053 = 2715
    reg chol: 1871 + 1415 + 2286 = 5572

C++
    reg chol: 3639
    block chol:  15725
 
 -->

## Range and RPM MR.CLAM


| Experiment | Timing (seconds) | # Poses | # Landmarks | # Range Measurements | # RPM Measurements |
| ---------- | ---------------: | ------- | ----------- | -------------------- | ------------------ |
| MR.CLAM 2  |          28.3035 | 31067   | 15          | 8392                 | 36465              |
| MR.CLAM 4  |          27.3805 | 26842   | 15          | 6561                 | 32503              |
| MR.CLAM 6  |           8.8067 | 11274   | 15          | 3537                 | 13121              |
| MR.CLAM 7  |           9.1253 | 13720   | 15          | 4004                 | 16056              |

