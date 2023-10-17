# Design Decisions & Documentation

This is living documentation of specific major decisions made in this repo.

## Choice of Optimization Libraries

We are aware of two C++ libraries which support Riemannian optimization:

- [David Rosen's optimization library](https://github.com/david-m-rosen/Optimization/)
- [ROPTLIB](https://github.com/whuang08/ROPTLIB)

We detail our observations on them below along with relevant pros/cons

### David Rosen's Optimization

**Capabilities**: This is a generic, header-only library that supports unconstrained Riemannian optimization. 

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

**Capabilities**: This is a typical C++ library which for Riemannian optimization. It seems to only support unconstrained Riemannian optimization. This is fine for us! 

**Software compatibility**: This comes with a handwritten `Makefile`, which is
cause for brief hesitation. We know of other systems 
([DPGO](https://github.com/mit-acl/dpgo)) that have used
`ROPTLIB` so that suggests it's easy enough to use. We are waiting to hear back
from Yulun Tian (author of DPGO) to get his opinion on ROPTLIB.

**Feature completeness**: This library supports a massive number of solvers, 
manifolds, and problems. Additionally, the documentation indicates that the
library has test utilities to empirically validate that the gradient and Hessian
operators are correctly implemented, similar to [manopt](https://www.manopt.org/).


### Takeaways

At the surface-level, the primary advantages of David Rosen's library are

- modern CMake and known to be easy to integrate
- implementation of LOBPCG solver
- slightly greater familiarity with codebase

The primary advantages of ROPTLIB are:

- more building blocks already exist so less work will (in theory) need to be done
- utilities to test correct implementation of gradients etc.
- in-depth user manual and documentation

Importantly, both libraries are viable with no clear issue in their usage.

### Uncertainties

**Performance**: it is unclear if either library is substantially more performant 
than the other. As
we'd like to deploy these systems on compute-constrained platforms this is an
important point to consider.

**Ease of use**: primarily, it is unclear how easy ROPTLIB is to use in external
libraries. It seems not too challenging, but is currently unclear how cross-platform
compatible it may be.
