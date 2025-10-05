"""Example showing how to build an InitializationMap, pack x0, and run solveCORA.

This script assumes `cora` extension is importable (via PYTHONPATH or installed).
"""
import numpy as np
import cora

# Parse an example problem
problem = cora.parsePyfgTextToProblem('examples/data/factor_graph_small.pyfg')

# Create an initialization map
inits = cora.InitializationMap()

# Example: set pose initialization for symbol 'x0' (character 'x', index 0)
sym = cora.Symbol('A', 0)
# rotation (d x d) and translation (d,)
d = problem.dim()
R = np.eye(d)
t = np.ones(d)

inits.set_pose_initialization(sym, R, t)

# pack x0 with relaxation_rank = problem.getRelaxationRank()
x0 = cora.getInitialMatrixFromInitMap(problem, inits)
print('Packed x0 shape:', x0.shape)
print(f"X0:\n{x0}")

# Run solver (small max_relaxation_rank for a quick run)
problem.updateProblemData()  # Ensure problem data is up to date
res, iters = cora.solveCORA(problem, x0, max_relaxation_rank=problem.getRelaxationRank(), verbose=False)
# print('Solver returned f =', res.f)
