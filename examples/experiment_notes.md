# Experiment Notes and Timing Results

## Original Experiments

| Experiment   | Timing (seconds) | # Poses | # Landmarks | # Range Measurements | # RPM Measurements |
| ------------ | ---------------- | ------- | ----------- | -------------------- | ------------------ |
| Plaza 1      | 10.5612          | -       | -           | -                    | -                  |
| Plaza 2      | 2.70548          | -       | -           | -                    | -                  |
| Single Drone | 2.26446          | -       | -           | -                    | -                  |
| TIERS        | 14.8041          | -       | -           | -                    | -                  |

## Range Only MR.CLAM

| Experiment | Timing (seconds) | # Poses | # Landmarks | # Range Measurements |
| ---------- | ---------------: | ------- | ----------- | -------------------- |
| MR.CLAM 2  |          255.757 | 31067   | 15          | 44857                |
| MR.CLAM 3a |          122.735 | 11568   | 15          | 21972                |
| MR.CLAM 3b |          31.3762 | 6271    | 15          | 8850                 |
| MR.CLAM 4  |          232.449 | 26842   | 15          | 39064                |
| MR.CLAM 5a |           6.0914 | 1080    | 15          | 1443                 |
| MR.CLAM 5b |          104.589 | 13955   | 15          | 20770                |
| MR.CLAM 5c |           103.16 | 15401   | 15          | 21800                |
| MR.CLAM 6  |           88.672 | 11274   | 15          | 16658                |
| MR.CLAM 7  |          99.3024 | 13720   | 15          | 20060                |


## Range and RPM MR.CLAM


| Experiment | Timing (seconds) | # Poses | # Landmarks | # Range Measurements | # RPM Measurements |
| ---------- | ---------------: | ------- | ----------- | -------------------- | ------------------ |
| MR.CLAM 2  |          35.3818 | 31067   | 15          | 8392                 | 36465              |
| MR.CLAM 3a |          16.4852 | 15568   | 15          | 3371                 | 18601              |
| MR.CLAM 3b |           5.8138 | 6271    | 15          | 1276                 | 7574               |
| MR.CLAM 4  |          31.3557 | 26842   | 15          | 6561                 | 32503              |
| MR.CLAM 5a |          1.26335 | 1080    | 15          | 316                  | 1127               |
| MR.CLAM 5b |          23.7146 | 13955   | 15          | 4815                 | 15955              |
| MR.CLAM 5c |          25.6283 | 15401   | 15          | 4906                 | 16894              |
| MR.CLAM 6  |          15.8511 | 11274   | 15          | 3537                 | 13121              |
| MR.CLAM 7  |          17.5916 | 13720   | 15          | 4004                 | 16056              |



<details>
Solving data/mrclam/range_and_rpm/mrclam2.pyfg

Solving problem at rank 3
Obtained solution with objective value: 7056.309528
Result is certified: 0 with eta: 0.007056 and theta: -0.005382

Solving problem at rank 4
Obtained solution with objective value: 7052.867355
Result is certified: 0 with eta: 0.007053 and theta: -0.003528

Solving problem at rank 5
Obtained solution with objective value: 7052.465391
Result is certified: 0 with eta: 0.007052 and theta: -0.005390

Solving problem at rank 6
Obtained solution with objective value: 7052.082564
Result is certified: 0 with eta: 0.007052 and theta: -0.014062

Solving problem at rank 7
Obtained solution with objective value: 7051.839907
Result is certified: 0 with eta: 0.007052 and theta: -0.013022

Solving problem at rank 8
Obtained solution with objective value: 7051.675059
Result is certified: 1 with eta: 0.007052 and theta: 0.000000

Projecting solution to rank 2 and refining.
Out of 31067 blocks, 0 have positive determinant. This is 0.000000% of the total.
Obtained solution with objective value: 7059.973997
Final solution is certified: 0 with eta: 0.007060 and theta: -0.083334
CORA took 35.3818 seconds

Solving data/mrclam/range_and_rpm/mrclam3a.pyfg

Solving problem at rank 3
Obtained solution with objective value: 1672.338772
Result is certified: 0 with eta: 0.001672 and theta: -0.001020

Solving problem at rank 4
Obtained solution with objective value: 1671.582965
Result is certified: 1 with eta: 0.001672 and theta: 0.000000

Projecting solution to rank 2 and refining.
Out of 15568 blocks, 205 have positive determinant. This is 1.316804% of the total.
Obtained solution with objective value: 1670.938871
Final solution is certified: 0 with eta: 0.001671 and theta: -0.001800
CORA took 16.4852 seconds

Solving data/mrclam/range_and_rpm/mrclam3b.pyfg

Solving problem at rank 3
Obtained solution with objective value: 1114.300903
Result is certified: 0 with eta: 0.001114 and theta: -0.000739

Solving problem at rank 4
Obtained solution with objective value: 1114.036899
Result is certified: 0 with eta: 0.001114 and theta: -0.001099

Solving problem at rank 5
Obtained solution with objective value: 1113.837982
Result is certified: 0 with eta: 0.001114 and theta: -0.000899

Solving problem at rank 6
Obtained solution with objective value: 1113.592745
Result is certified: 1 with eta: 0.001114 and theta: 0.000000

Projecting solution to rank 2 and refining.
Out of 6271 blocks, 6239 have positive determinant. This is 99.489715% of the total.
Obtained solution with objective value: 1114.177407
Final solution is certified: 0 with eta: 0.001114 and theta: -0.011011
CORA took 5.8138 seconds

Solving data/mrclam/range_and_rpm/mrclam4.pyfg

Solving problem at rank 3
Obtained solution with objective value: 4886.186750
Result is certified: 0 with eta: 0.004886 and theta: -0.665794

Solving problem at rank 4
Obtained solution with objective value: 4885.104424
Result is certified: 0 with eta: 0.004885 and theta: -3.473149

Solving problem at rank 5
Obtained solution with objective value: 4157.993238
Result is certified: 0 with eta: 0.004158 and theta: -0.002200

Solving problem at rank 6
Obtained solution with objective value: 4157.955639
Result is certified: 0 with eta: 0.004158 and theta: -0.002302

Solving problem at rank 7
Obtained solution with objective value: 4157.944573
Result is certified: 0 with eta: 0.004158 and theta: -0.005734

Solving problem at rank 8
Obtained solution with objective value: 4157.941310
Result is certified: 1 with eta: 0.004158 and theta: 0.000000

Projecting solution to rank 2 and refining.
Out of 26842 blocks, 26842 have positive determinant. This is 100.000000% of the total.
Obtained solution with objective value: 4158.088361
Final solution is certified: 0 with eta: 0.004158 and theta: -0.061544
CORA took 31.3557 seconds

Solving data/mrclam/range_and_rpm/mrclam5a.pyfg

Solving problem at rank 3
Obtained solution with objective value: 58.890538
Result is certified: 1 with eta: 0.000059 and theta: 0.000000

Projecting solution to rank 2 and refining.
Out of 1080 blocks, 534 have positive determinant. This is 49.444444% of the total.
Obtained solution with objective value: 75.280277
Final solution is certified: 0 with eta: 0.000075 and theta: -0.013838
CORA took 1.26335 seconds

Solving data/mrclam/range_and_rpm/mrclam5b.pyfg

Solving problem at rank 3
Obtained solution with objective value: 1550.229908
Result is certified: 0 with eta: 0.001550 and theta: -0.000791

Solving problem at rank 4
Obtained solution with objective value: 1547.710450
Result is certified: 0 with eta: 0.001548 and theta: -0.001178

Solving problem at rank 5
Obtained solution with objective value: 1547.190794
Result is certified: 0 with eta: 0.001547 and theta: -0.000813

Solving problem at rank 6
Obtained solution with objective value: 1547.093495
Result is certified: 0 with eta: 0.001547 and theta: -0.006812

Solving problem at rank 7
Obtained solution with objective value: 1547.060433
Result is certified: 0 with eta: 0.001547 and theta: -0.009454

Solving problem at rank 8
Obtained solution with objective value: 1547.053852
Result is certified: 0 with eta: 0.001547 and theta: -0.001430

Solving problem at rank 9
Obtained solution with objective value: 1546.918200
Result is certified: 0 with eta: 0.001547 and theta: -0.006338

Solving problem at rank 10
Obtained solution with objective value: 1546.868110
Result is certified: 0 with eta: 0.001547 and theta: -0.013080

Projecting solution to rank 2 and refining.
Out of 13955 blocks, 13955 have positive determinant. This is 100.000000% of the total.
Obtained solution with objective value: 1555.616078
Final solution is certified: 0 with eta: 0.001556 and theta: -0.036298
CORA took 23.7146 seconds

Solving data/mrclam/range_and_rpm/mrclam5c.pyfg

Solving problem at rank 3
Obtained solution with objective value: 2203.103105
Result is certified: 0 with eta: 0.002203 and theta: -0.105263

Solving problem at rank 4
Obtained solution with objective value: 1695.057436
Result is certified: 0 with eta: 0.001695 and theta: -0.001158

Solving problem at rank 5
Obtained solution with objective value: 1694.964185
Result is certified: 0 with eta: 0.001695 and theta: -0.000856

Solving problem at rank 6
Obtained solution with objective value: 1694.744769
Result is certified: 0 with eta: 0.001695 and theta: -0.006562

Solving problem at rank 7
Obtained solution with objective value: 1694.692664
Result is certified: 0 with eta: 0.001695 and theta: -0.001176

Solving problem at rank 8
Obtained solution with objective value: 1694.524673
Result is certified: 0 with eta: 0.001695 and theta: -0.010525

Solving problem at rank 9
Obtained solution with objective value: 1694.479860
Result is certified: 0 with eta: 0.001694 and theta: -0.000948

Solving problem at rank 10
Obtained solution with objective value: 1694.346055
Result is certified: 0 with eta: 0.001694 and theta: -0.008909

Projecting solution to rank 2 and refining.
Out of 15401 blocks, 0 have positive determinant. This is 0.000000% of the total.
Obtained solution with objective value: 1700.698467
Final solution is certified: 0 with eta: 0.001701 and theta: -0.100971
CORA took 25.6283 seconds

Solving data/mrclam/range_and_rpm/mrclam6.pyfg

Solving problem at rank 3
Obtained solution with objective value: 3138.219718
Result is certified: 0 with eta: 0.003138 and theta: -0.001662

Solving problem at rank 4
Obtained solution with objective value: 3137.212063
Result is certified: 0 with eta: 0.003137 and theta: -0.008975

Solving problem at rank 5
Obtained solution with objective value: 3136.325024
Result is certified: 0 with eta: 0.003136 and theta: -0.014310

Solving problem at rank 6
Obtained solution with objective value: 3136.051568
Result is certified: 0 with eta: 0.003136 and theta: -0.008982

Solving problem at rank 7
Obtained solution with objective value: 3135.469158
Result is certified: 0 with eta: 0.003135 and theta: -0.017083

Solving problem at rank 8
Obtained solution with objective value: 3135.281277
Result is certified: 0 with eta: 0.003135 and theta: -0.004617

Solving problem at rank 9
Obtained solution with objective value: 3135.023159
Result is certified: 0 with eta: 0.003135 and theta: -0.001762

Solving problem at rank 10
Obtained solution with objective value: 3134.819336
Result is certified: 0 with eta: 0.003135 and theta: -0.001664

Projecting solution to rank 2 and refining.
Out of 11274 blocks, 11274 have positive determinant. This is 100.000000% of the total.
Obtained solution with objective value: 3152.346062
Final solution is certified: 0 with eta: 0.003152 and theta: -0.037511
CORA took 15.8511 seconds

Solving data/mrclam/range_and_rpm/mrclam7.pyfg

Solving problem at rank 3
Obtained solution with objective value: 3038.745942
Result is certified: 0 with eta: 0.003039 and theta: -0.001520

Solving problem at rank 4
Obtained solution with objective value: 3038.655156
Result is certified: 0 with eta: 0.003039 and theta: -0.008282

Solving problem at rank 5
Obtained solution with objective value: 3038.561468
Result is certified: 0 with eta: 0.003039 and theta: -0.005450

Solving problem at rank 6
Obtained solution with objective value: 3038.480235
Result is certified: 0 with eta: 0.003038 and theta: -0.005523

Solving problem at rank 7
Obtained solution with objective value: 3038.411905
Result is certified: 0 with eta: 0.003038 and theta: -0.008041

Solving problem at rank 8
Obtained solution with objective value: 3038.345194
Result is certified: 0 with eta: 0.003038 and theta: -0.003883

Solving problem at rank 9
Obtained solution with objective value: 3038.316805
Result is certified: 0 with eta: 0.003038 and theta: -0.006311

Solving problem at rank 10
Obtained solution with objective value: 3038.172892
Result is certified: 0 with eta: 0.003038 and theta: -0.006733

Projecting solution to rank 2 and refining.
Out of 13720 blocks, 13714 have positive determinant. This is 99.956268% of the total.
Obtained solution with objective value: 3110.425700
Final solution is certified: 0 with eta: 0.003110 and theta: -0.066661
CORA took 17.5916 seconds
</details>
