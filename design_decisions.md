# Design Decisions & Documentation

This is living documentation of specific major decisions made in this repo.

## Choice of Optimization Libraries

We are aware of two C++ libraries which support Riemannian optimization:

- [David Rosen's optimization library](https://github.com/david-m-rosen/Optimization/)
- [ROPTLIB](https://github.com/whuang08/ROPTLIB)

We detail our observations on them below along with relevant pros/cons

### David Rosen's Optimization

**Capabilities**: This is a generic, header-only library that supports both convex and unconstrained Riemannian optimization. 

**Software compatibility**: This is packaged with relatively modern CMake. It's easy to run and integrate into a system.

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

