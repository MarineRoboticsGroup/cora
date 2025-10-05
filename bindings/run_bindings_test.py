# Minimal runtime test for the CORA Python bindings.
# It expects the built extension to be importable (via PYTHONPATH or the install script).

import sys
import numpy as np

try:
    import cora
except Exception as e:
    print('FAILED to import cora:', e)
    sys.exit(2)

print('Imported cora from', getattr(cora, '__file__', '<unknown>'))

# Try parsing a small example shipped with the repo
example_path = 'examples/data/plaza1.pyfg'
try:
    problem = cora.parsePyfgTextToProblem(example_path)
except Exception as e:
    print('FAILED to parse example file:', e)
    sys.exit(3)

print('Parsed problem. dim=', problem.dim())

# Build a zero-initialization matching the expected data matrix size and relaxation rank.
# Use problem.getDataMatrixSize() x problem.getRelaxationRank(). Access via getattr to
# avoid static analysis issues in editors when using stubs.
try:
    get_rows = getattr(problem, 'getDataMatrixSize', None)
    get_cols = getattr(problem, 'getRelaxationRank', None)
    if not callable(get_rows) or not callable(get_cols):
        raise RuntimeError('problem object does not expose expected size getters')

    rows = int(get_rows()) # type: ignore
    cols = int(get_cols()) # type: ignore
    x0 = np.zeros((rows, cols), dtype=np.float64)
except Exception as e:
    print('FAILED to get matrix sizes from problem:', e)
    sys.exit(4)

# Run the solver with very lightweight settings (max_relaxation_rank small to keep runtime low)
try:
    max_rank = cols
    res, iters = cora.solveCORA(problem, x0, max_relaxation_rank=max_rank, verbose=False)
    print('Solver returned objective f=', res.f)
    print('Returned', len(iters), 'logged iterates')
except Exception as e:
    print('FAILED running solver:', e)
    sys.exit(5)

print('Bindings smoke test PASSED')
