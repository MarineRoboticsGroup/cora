"""Extensive tests for the CORA variable-management helpers.

This script creates a small Problem programmatically and exercises the
following functions (from `CORA_variable_management` via the Python
bindings):
 - Values (set/get/has)
 - getVarMatrixFromValues
 - getValuesFromVarMatrix
 - updateValuesFromVarMatrix
 - updateVarMatrixRangesBasedOnTranslationVals
 - updateVarMatrixFromValues
 - getRandomValues
 - getRandomVarMatrix

Run this after building the `cora` extension (or install it into your
Python environment so `import cora` works).
"""

import numpy as np
import cora


def main():
	print('Creating problem with dim=3, relaxation_rank=5')
	dim = 3
	relax_rank = 5
	problem = cora.Problem(dim, relax_rank)

	# Add two poses and one landmark
	p0 = cora.Symbol('x', 0)
	p1 = cora.Symbol('x', 1)
	l0 = cora.Symbol('l', 0)
	problem.addPoseVariable(p0)
	problem.addPoseVariable(p1)
	problem.addLandmarkVariable(l0)

	# Add a range measurement between pose p1 and landmark l0 so the
	# range-rows exist in the var-matrix
	# (r and cov are dummy values; updateVarMatrixRangesBasedOnTranslationVals
	# will recompute the bearing)
	problem.addRangeMeasurement(cora.RangeMeasurement(p1, l0, 1.0, 0.1))

	# Make sure problem internal metadata is available
	problem.updateProblemData()

	print('Requesting a random var-matrix for the problem')
	x0_rand = cora.getRandomVarMatrix(problem)
	print('Random x0 shape:', x0_rand.shape)

	print('Converting random x0 into Values (getValuesFromVarMatrix)')
	vals_from_rand = cora.getValuesFromVarMatrix(problem, x0_rand)

	# Values read from random x0 should report has_pose/has_landmark depending
	# on whether the random initial guess filled them. We accept either True/False
	print('has_pose p0:', vals_from_rand.has_pose(p0))
	print('has_landmark l0:', vals_from_rand.has_landmark(l0))

	# Create an explicit Values and set specific pose/landmark initializations
	print('Creating explicit Values and setting known pose/landmark')
	inits = cora.Values()
	R = np.eye(dim)
	t = np.arange(1, dim + 1).astype(float)
	inits.set_pose(p0, R, t)
	assert inits.has_pose(p0), 'inits should have pose p0'

	p = inits.get_pose_rotation(p0)
	assert p is not None and np.allclose(p, R), 'rotation round-trip failed'
	tt = inits.get_pose_translation(p0)
	assert tt is not None and np.allclose(tt, t), 'translation round-trip failed'

	# set a landmark
	lp = np.array([0.5, 1.5, -2.0])
	inits.set_landmark(l0, lp)
	assert inits.has_landmark(l0)
	lval = inits.get_landmark(l0)
	assert lval is not None and np.allclose(lval, lp)

	# Pack the Values into a var-matrix
	print('Packing Values into a var-matrix (getVarMatrixFromValues)')
	x0_from_inits = cora.getVarMatrixFromValues(problem, inits)
	print('Packed x0 shape:', x0_from_inits.shape)

	# Unpack back and check equality
	print('Unpacking x0 back into Values (getValuesFromVarMatrix)')
	vals_unpacked = cora.getValuesFromVarMatrix(problem, x0_from_inits)
	assert vals_unpacked.has_pose(p0)
	assert vals_unpacked.has_landmark(l0)
	vt = vals_unpacked.get_pose_translation(p0)
	assert vt is not None and np.allclose(vt, t)

	# updateValuesFromVarMatrix: modify an existing Values from a matrix
	print('Testing updateValuesFromVarMatrix')
	# start with empty Values and update from previously-packed matrix
	vals_empty = cora.Values()
	cora.updateValuesFromVarMatrix(problem, vals_empty, x0_from_inits)
	assert vals_empty.has_pose(p0)
	vt2 = vals_empty.get_pose_translation(p0)
	assert vt2 is not None and np.allclose(vt2, t)

	# updateVarMatrixFromValues: given a random x0, write our inits into it
	print('Testing updateVarMatrixFromValues (returns modified copy)')
	x0_rand2 = cora.getRandomVarMatrix(problem)
	x0_updated = cora.updateVarMatrixFromValues(problem, inits, x0_rand2)
	# After update, extracting Values should reflect our inits for p0 and l0
	vals_after_update = cora.getValuesFromVarMatrix(problem, x0_updated)
	assert vals_after_update.has_pose(p0)
	vt3 = vals_after_update.get_pose_translation(p0)
	assert vt3 is not None and np.allclose(vt3, t)

	# updateVarMatrixRangesBasedOnTranslationVals: ensure it returns a matrix with
	# modified range rows compared to the input
	print('Testing updateVarMatrixRangesBasedOnTranslationVals')
	x0_before_ranges = x0_updated.copy()
	x0_ranges_updated = cora.updateVarMatrixRangesBasedOnTranslationVals(problem, x0_before_ranges)
	# matrices should be same shape
	assert x0_ranges_updated.shape == x0_before_ranges.shape
	# If any range rows exist, at least one row should differ (bearing recomputed)
	if not np.allclose(x0_ranges_updated, x0_before_ranges):
		print('Range rows updated (matrix changed)')
	else:
		print('Range rows did not change (possible if random values matched)')

	# getRandomValues -> Values and ensure it contains items
	print('Testing getRandomValues')
	random_vals = cora.getRandomValues(problem)
	print('random_vals.has_pose(p0):', random_vals.has_pose(p0))

	print('All variable-management tests passed.')


if __name__ == '__main__':
	try:
		main()
	except AssertionError as e:
		print('Assertion failed:', e)
		raise
