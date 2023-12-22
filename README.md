# Certifiably Correct Range-Aided SLAM (CORA)

This is the official repository for the paper ["Certifiably Correct Range-Aided SLAM"](https://arxiv.org/abs/2302.11614) by
[Alan Papalia](https://alanpapalia.github.io), Andrew Fishberg, Brendan O'Neill, [Jonathan P. How](https://www.mit.edu/~jhow/),
[David M. Rosen](https://david-m-rosen.github.io/) and [John J. Leonard](https://meche.mit.edu/people/faculty/JLEONARD@MIT.EDU).

If you prefer to work in MATLAB, you can look at our (deprecated) implementation in [cora-matlab](https://github.com/MarineRoboticsGroup/cora-matlab).

## Building

Install dependencies
```bash
sudo apt-get install build-essential cmake-gui libeigen3-dev liblapack-dev libblas-dev libsuitesparse-dev -y
```

Install submodules and build with CMake
```bash
git clone git@github.com:MarineRoboticsGroup/cora-plus-plus.git
cd cora-plus-plus
git submodule update --init
mkdir build
cd build
cmake ..
make -j
```

## Usage

For now look in our `examples/` directory for how to use this library.

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

## Let there be Viz

To build with visualization there is a bit more set up needed. The following
terminal commands should get you up and running with visualization.

```bash
# install Pangolin
git clone git@github.com:stevenlovegrove/Pangolin.git
cd Pangolin
git checkout dd801d2 # this is Tag 0.6 - was most recent tag we were able to build
mkdir build
cd build
cmake ..
make -j
sudo make install

# install GTSAM 4.2
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt-get update
sudo apt-get install libgtsam-dev libgtsam-unstable-dev
```

## Code Profiling

Additionally, if you want to run profiling you will need to install the Google
Performance Tools

```bash
sudo apt-get install libgoogle-perftools-dev google-perftools graphviz
```

## With some help from our friends

This implementation relies heavily on the highly performant [optimization
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
