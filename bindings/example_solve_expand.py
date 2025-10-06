"""Comprehensive example that builds a problem, solves it, expands it, and re-solves.

This script exercises the solver and variable-management bindings:
 - create Problem, add variables and measurements
 - obtain random initialization and solve
 - extract solution into Values
 - add more variables/measurements to the same Problem
 - build a new initialization using previous Values and resolve

Run after building and installing the `cora` extension (or with PYTHONPATH).
"""

import numpy as np
import cora

# set numpy print format to two decimal places for easier reading
np.set_printoptions(precision=2, suppress=True)


def add_core_variables_and_measurements(problem: cora.Problem):
    # Add two poses and one landmark; add priors & measurements to make the
    # problem well-constrained for a quick solve.
    p0 = cora.Symbol('x', 0)
    p1 = cora.Symbol('x', 1)
    l0 = cora.Symbol('l', 0)

    problem.addPoseVariable(p0)
    problem.addPoseVariable(p1)
    problem.addLandmarkVariable(l0)

    d = problem.dim()
    R_id = np.eye(d)
    t0 = np.zeros(d)
    cov_pose = np.eye(d) * 1e-2

    # Anchor p0 with a pose prior
    prior = cora.PosePrior(p0, R_id, t0, cov_pose)
    problem.addPosePrior(prior)

    # Relative pose measurement between p0 and p1
    rel_t = np.array([0.5] * d)
    rel_cov = np.eye(d) * 1e-2
    rel_meas = cora.RelativePoseMeasurement(p0, p1, R_id, rel_t, rel_cov)
    problem.addRelativePoseMeasurement(rel_meas)

    # Landmark prior (optional) and a relative pose-landmark measurement
    lp = np.array([1.0] * d)
    cov_land = np.eye(d) * 1e-2
    lprior = cora.LandmarkPrior(l0, lp, cov_land)
    problem.addLandmarkPrior(lprior)

    # Range measurement between p1 and l0
    range_meas = cora.RangeMeasurement(p1, l0, 1.0, 0.01)
    problem.addRangeMeasurement(range_meas)


def solve_and_extract(problem: cora.Problem, x0: np.ndarray, expect_num_poses: int):
    problem.updateProblemData()
    print('Solving problem (expecting', expect_num_poses, 'poses)')
    res, iters = cora.solveCORA(problem, x0, max_relaxation_rank=problem.getRelaxationRank(), verbose=False)
    print('Solver objective f =', res.f)

    Y = res.x
    # Extract poses and landmarks from solution
    poses = []
    for i in range(expect_num_poses):
        s = cora.Symbol('x', i)
        pr = cora.extractPose(problem, Y, s)
        Rsol, tsol = pr[0], pr[1]

        # check that the rotation is valid
        should_be_identity = Rsol @ Rsol.T
        assert np.allclose(should_be_identity, np.eye(problem.dim()), atol=1e-5), 'extracted rotation is not orthogonal'
        assert np.isclose(np.linalg.det(Rsol), 1.0, atol=1e-5), 'extracted rotation is not proper (det!=1)'

        poses.append((Rsol, tsol))

    # Extract any landmarks present
    landmarks = []
    # We'll attempt to extract landmark 0 if present
    try:
        l0 = cora.Symbol('l', 0)
        pv = cora.extractPoint(problem, Y, l0)
        print('Landmark l0:', pv)
        landmarks.append(pv)
    except Exception:
        # Some problems may not have landmarks — ignore
        pass

    return res, poses, landmarks


def main():
    dim = 3
    relax_rank = 6
    problem = cora.Problem(dim, relax_rank)
    problem.setFormulation(cora.Formulation.Implicit)

    print('Creating initial problem with 2 poses and 1 landmark')
    add_core_variables_and_measurements(problem)
    print("Updating problem data")
    problem.updateProblemData()

    # get a random initialization
    print(f"Requesting a random initial guess x0 of shape ({problem.getExpectedVariableSize()}, {problem.getRelaxationRank()})")
    x0 = cora.getRandomVarMatrix(problem)

    # Solve the original small problem
    print(f"Solving original problem")
    res, poses, landmarks = solve_and_extract(problem, x0, expect_num_poses=2)

    # Convert solver solution into a Values map
    print('Converting solution matrix into Values')
    vals = cora.getValuesFromVarMatrix(problem, res.x)

    # Expand the problem: add another pose and landmark and measurements
    print('Expanding problem: adding pose x2 and landmark l1, and measurements')
    p2 = cora.Symbol('x', 2)
    l1 = cora.Symbol('l', 1)
    problem.addPoseVariable(p2)
    problem.addLandmarkVariable(l1)

    # Connect p2 to p1 with a relative pose measurement
    R_id = np.eye(dim)
    rel_t2 = np.array([0.3] * dim)
    problem.addRelativePoseMeasurement(cora.RelativePoseMeasurement(cora.Symbol('x', 1), p2, R_id, rel_t2, np.eye(dim) * 1e-2))

    # Connect new landmark to p2 with a relative pose-landmark measurement
    problem.addRelativePoseLandmarkMeasurement(cora.RelativePoseLandmarkMeasurement(p2, l1, np.array([0.2]*dim), np.eye(dim) * 1e-2))

    # Add a range measurement between p2 and l1
    problem.addRangeMeasurement(cora.RangeMeasurement(p2, l1, 0.5, 0.01))

    problem.updateProblemData()

    # Now build a new x0 using Values from the previous solution (existing symbols)
    # getVarMatrixFromValues will fill unspecified new variables randomly
    print('Packing previous Values into a new x0 for the expanded problem')
    new_x0 = cora.getVarMatrixFromValues(problem, vals)
    print('New x0 shape:', new_x0.shape)

    # Optionally update ranges based on translation values
    new_x0 = cora.updateVarMatrixRangesBasedOnTranslationVals(problem, new_x0)

    # Solve the expanded problem
    res2, poses2, landmarks2 = solve_and_extract(problem, new_x0, expect_num_poses=3)

    print('Expanded problem solved. Final objective:', res2.f)


if __name__ == '__main__':
    main()
