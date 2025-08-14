import cora_py
import numpy as np

# Create a problem
problem = cora_py.Problem(3, 5)

# Add some variables
problem.addPoseVariable(cora_py.Symbol('a', 0))
problem.addPoseVariable(cora_py.Symbol('a', 1))
problem.addLandmarkVariable(cora_py.Symbol('l', 0))

# Add some measurements
R = np.eye(3)
t = np.array([1, 0, 0])
cov = np.eye(6)
problem.addRelativePoseMeasurement(cora_py.Symbol('a', 0), cora_py.Symbol('a', 1), R, t, cov)

# Update problem data
problem.updateProblemData()

# Solve the problem
result = problem.solve()

# Get the solution
solution = problem.getTranslationExplicitSolution(result)

print("Solution:")
print(solution)
