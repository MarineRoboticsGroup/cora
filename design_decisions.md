# Design Decisions & Documentation

This is living documentation of specific major decisions made in this repo.

## Choice of Optimization Libraries

We are aware of three C++ libraries which support Riemannian optimization:

- [David Rosen's optimization library](https://github.com/david-m-rosen/Optimization/)
- [ROPTLIB](https://github.com/whuang08/ROPTLIB)
- [GTSAM](https://github.com/borglab/gtsam)

Implementing some of the desired subroutines for CORA in GTSAM is certainly
doable but may be a bit painful due to some of the design decisions in GTSAM. We
suspect that this would still be a performant solution, but the integration work
may be a bit more involved than the other two options. As a result, we currently
do not consider GTSAM further.

We detail our observations on David's library and ROPTLIB below along with relevant pros/cons

### David Rosen's Optimization

**Capabilities**: This is a generic, header-only library that supports unconstrained Riemannian optimization. This has obtained highly efficient performance
in pose-graph SLAM problems, which is a good sign for our use case.

**Software compatibility**: This is packaged with relatively modern CMake. It's easy to run and integrate into a system. We have personally done this before.

**Feature completeness**: This library supports both truncated-Newton trust-region (TNT) and least-squares (TNLS) solvers.
The solvers are provided a set of operations (e.g., choice of metric and retraction) that represent
the underlying optimization problem. This provides completeness, but likely means that we will need to define
much of our functionality ourselves as opposed to piecing together the correct building blocks. See example `TNLS`
function definition below.

```c++
TNLS(const Mapping<VariableX, VectorY, Args...> &F,
     const JacobianPairFunction<VariableX, TangentX, VectorY> &J,
     const RiemannianMetric<VariableX, TangentX, Scalar, Args...> &metric_X,
     const LinearAlgebra::InnerProduct<VectorY, Scalar, Args...>
         &inner_product_Y,
     const Retraction<VariableX, TangentX, Args...> &retract_X,
     const VariableX &x0, Args &...args,
     const std::optional<TNLSPreconditioner<VariableX, TangentX, Args...>>
         &precon = std::nullopt,
     const TNLSParams<Scalar> &params = TNLSParams<Scalar>(),
     const std::optional<
         TNLSUserFunction<VariableX, TangentX, VectorY, Scalar, Args...>>
         &user_function = std::nullopt)
```

**Added features**: this library also supports a LOBPCG eigensolver, this is a key advantage.

### ROPTLIB

**Capabilities**: This is a typical C++ library which for Riemannian
optimization. It seems to only support unconstrained Riemannian optimization.
Word of mouth has suggested that ROPTLIB is slower than GTSAM, but we don't have
any data on hand.


**Software compatibility**: This comes with a handwritten `Makefile`, which is
cause for brief hesitation. Others who have used ROPTLIB in a robotics stack
commented that it was a decent bit of work to get it properly integrated via
CMake.

**Feature completeness**: This library supports a massive number of solvers,
manifolds, and problems. Additionally, the documentation indicates that the
library has test utilities to empirically validate that the gradient and Hessian
operators are correctly implemented, similar to [manopt](https://www.manopt.org/).

### Takeaways

At the surface-level, the primary advantages of David Rosen's library are

- modern CMake and easy to integrate
- implementation of LOBPCG solver
- slightly greater familiarity with codebase

The primary advantages of ROPTLIB are:

- more building blocks already exist so less work will (in theory) need to be done
- utilities to test correct implementation of gradients etc.
- in-depth user manual and documentation

**Decision**: Importantly, both libraries are viable with no outright issue in their usage.
As Dave's library is known to be easier to integrate, has a LOBPCG solver, and is
likely more performant, we will use that library for now. If we find that we need
more functionality, we can always switch to ROPTLIB later and update this documentation
accordingly.
