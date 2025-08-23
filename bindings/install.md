Some commands I've needed to get the Python bindings to build:

```bash
# Note: you can use either conda or mamba; mamba is just faster
mamba install -c conda-forge eigen suitesparse cmake ninja
CMAKE_PREFIX_PATH="$CONDA_PREFIX" pip install -e . -v
```

Alterntaive:

```bash
CMAKE_PREFIX_PATH=/usr \
CMAKE_IGNORE_PATH="$CONDA_PREFIX/include;$CONDA_PREFIX/lib;$CONDA_PREFIX/lib64" \
CMAKE_ARGS="-DBUILD_PYTHON_BINDINGS=ON" \
pip install -e . -v
```
