# Certifiably Correct Range-Aided SLAM (CORA)

This is the official repository for the paper ["Certifiably Correct Range-Aided SLAM"](https://arxiv.org/abs/2302.11614) by
[Alan Papalia](https://alanpapalia.github.io), Andrew Fishberg, Brendan O'Neill, [Jonathan P. How](https://www.mit.edu/~jhow/),
[David M. Rosen](https://david-m-rosen.github.io/) and [John J. Leonard](https://meche.mit.edu/people/faculty/JLEONARD@MIT.EDU).

If you prefer to work in MATLAB, you can look at our (deprecated) implementation in [cora-matlab](https://github.com/MarineRoboticsGroup/cora-matlab).

## Quick Demo

Here you can see CORA solving the Plaza 2 data set from a random initialization! You can recreate this with our
example in `examples/cora_vis_tests.cpp` by uncommenting `line 12` and switching the filepath to `"./bin/example_data/factor_graph.pyfg");`

![Plaza 2](https://github.com/MarineRoboticsGroup/cora/assets/17442843/1a0ad0b0-a554-4248-9b92-f52815d1cd34)

## Building with Conan

This project uses Conan for package management. To build the project, you will need to install Conan.

```bash
pip install conan
```

Then, you can build the project with CMake.

```bash
git clone git@github.com:MarineRoboticsGroup/cora-plus-plus.git
cd cora-plus-plus
git submodule update --init
conan install . --output-folder=build --build=missing
cd build
cmake ..
make -j
```

### Build Options

You can customize the build by passing options to Conan when running `conan install`. For example, to build without visualization and with tests enabled, you can run:

```bash
conan install . --output-folder=build --build=missing -o cora/enable_visualization=False -o cora/build_tests=True
```

The available options are:

* `shared`: Build as a shared library (`True`/`False`, default: `False`)
* `fPIC`: Enable position-independent code (`True`/`False`, default: `True`)
* `enable_vectorization`: Enable vectorized instruction sets (`True`/`False`, default: `True`)
* `enable_openmp`: Enable OpenMP parallelization (`True`/`False`, default: `False`)
* `enable_profiling`: Enable code profiling with gperftools (`True`/`False`, default: `False`)
* `enable_visualization`: Enable visualization module (`True`/`False`, default: `True`)
* `perform_experiments`: Run experiments from the paper (`True`/`False`, default: `True`)
* `build_examples`: Build C++ examples (`True`/`False`, default: `True`)
* `build_python_bindings`: Build Python bindings (`True`/`False`, default: `True`)
* `build_tests`: Build tests (`True`/`False`, default: `True`)


## Usage

For now look in our `examples/` directory for how to use this library.

If you want to inspect the data we have, you can use the
[PyFactorGraph library](https://github.com/MarineRoboticsGroup/PyFactorGraph).
We have set up a small python script `examples/data_viz.py` to help you visualize
the date, but you will need to install the PyFactorGraph library to use it.

```bash
pip install git+https://github.com/MarineRoboticsGroup/PyFactorGraph
cd examples
python data_viz.py
```

Below is our visualization of the "Single Drone" data set we use in our paper.
We visualize both the odometry train and the ground truth position of the drone.
Range measurements are shown with the dashed grey lines between the drone and
the ground station.

<img src="https://github.com/MarineRoboticsGroup/cora/assets/17442843/41c3eb0e-d6fb-4f1e-95df-8b9e0a1ed6ec" width="600">

https://github.com/MarineRoboticsGroup/cora/assets/17442843/05fdf949-3e23-4f3a-83a3-8eb1e706c2e5

## Contributing

Any contributions should pass all checks in our `.pre-commit-config.yaml` file.
To install the pre-commit hooks, run `pre-commit install` in the root directory
of this repository.

You may need to install some dependencies to get the pre-commit hooks to work.

```
pip install pre-commit
sudo apt-get install cppcheck
cd /path/to/cora
pre-commit install
```



## With some help from our friends

This implementation relies heavily on this highly performant [optimization
library](https://github.com/david-m-rosen/Optimization) by David M. Rosen!


## Contributors

* [Tim Magoun](https://www.linkedin.com/in/timmagoun/)
* [Alan Papalia](https://alanpapalia.github.io/)


## References

If you use this code in your research, please cite the following paper:

```
@article{papalia2023certifiably,
  title={Certifiably Correct Range-Aided SLAM},
  author={Papalia, Alan and Fishberg, Andrew and O'Neill, Brendan W. and How, Jonathan P. and Rosen, David M. and Leonard, John J.},
  journal={arXiv preprint arXiv:2302.11614},
  year={2023}
}
```

## Troubleshooting

**Error** `error while loading shared libraries: libpango_windowing.so: cannot open shared object file: No such file or directory`

**Solution**: run `sudo ldconfig`
