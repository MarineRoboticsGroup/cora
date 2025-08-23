# Python Bindings for CORA

This document describes how to build and use the Python bindings for the CORA library.

## Building the Python Bindings

The Python bindings are built using `pybind11` and CMake. The following steps will guide you through the process:

1.  **Navigate to the project's root directory.**

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Run CMake to configure the build:**
    ```bash
    cmake ..
    ```
    This will automatically detect your Python installation and configure the build accordingly.

4.  **Compile the code:**
    ```bash
    make
    ```
    This will compile the C++ code and create the Python module file (e.g., `cora_py.so`).

## Using the Python Bindings

Once the build is complete, the Python module will be located in the `build/lib` directory. To use it, you can either:

1.  **Install the module:**
    ```bash
    make install
    ```
    This will install the module into your Python's `site-packages` directory, making it available to all your Python scripts.

2.  **Add the build directory to your `PYTHONPATH`:**
    ```bash
    export PYTHONPATH=$PYTHONPATH:/path/to/cora-plus-plus/build/lib
    ```
    This will make the module available in your current shell session.

Once the module is in your Python path, you can import it in your Python scripts like this:

```python
import cora_py

# Now you can use the functions and classes defined in the C++ code
problem = cora_py.Problem(3, 3)
# ...
```
