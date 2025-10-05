# Python Bindings for CORA

This directory contains the Python bindings for the CORA library, enabling you to use CORA's C++ functionality from Python via pybind11.

## Prerequisites
- C++ compiler (GCC or Clang)
- CMake (>=3.14 recommended)
- Python 3.6+
- [pybind11](https://github.com/pybind/pybind11) (usually handled by CMake)
- CORA dependencies (Eigen, etc.)

## Building the Python Bindings

1. **From the project root, configure and build with CMake:**

   ```bash
   mkdir -p build
   cd build
   cmake .. -DBUILD_PYTHON_BINDINGS=ON
   make -j$(nproc)
   ```
   This will build the shared library (e.g., `cora.cpython-<version>-<platform>.so`) in `build/lib/`.


2. **(Optional) Make the Python module importable in your environment:**

Because this project doesn’t currently ship a `setup.py/pyproject.toml`, the easiest ways to use the built module are:

- Option A (recommended): Write a .pth file into your environment’s site-packages
   - This repo ships a helper script that does exactly that.

   ```bash
   # From the project root after building
   ./bindings/install_cora_python.sh
   ```

   The script writes a `cora_local.pth` into your active Python’s site-packages,
   pointing at `build/lib`, so `import cora` works anywhere. You can pass a
   specific build dir or Python:

   ```bash
   ./bindings/install_cora_python.sh /path/to/build/lib python
   ```

   **Does this work even if I update CORA and rebuild?**
   Yes, as long as you rebuild the bindings in the same build directory, the
   `.pth` file will continue to point to the correct location of the updated
   shared object file. Just ensure that you run the build commands again after
   making changes to CORA or its bindings.

- Option B: Use PYTHONPATH during development (no files modified)

   ```bash
   # From the project root
   export PYTHONPATH="$(pwd)/build/lib:${PYTHONPATH}"
   python -c "import cora, sys; print('cora from', cora.__file__)"
   ```

- Option C: Copy the built shared object into site-packages directly

   ```bash
   # Find your site-packages
   python - <<'PY'
import sysconfig
print(sysconfig.get_paths()['purelib'])
PY
   # Copy the .so from build/lib into the path printed above
   # Example:
   cp build/lib/cora.cpython-*.so "$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')/"
   ```

Pick the option that fits your workflow. Option A is easy to undo by removing the `cora_local.pth` file.


## Using the Python Bindings

In your Python code:

```python
import cora
# Now you can use CORA's classes and functions
```

## Modifying the Bindings

To expose new C++ classes or functions to Python:

1. **Edit `cora_bindings.cpp`:**
   - To expose a new function you write yourself:
    ```cpp
    // In cora_bindings.cpp
    int add(int a, int b) { return a + b; }

    PYBIND11_MODULE(cora, m) {
       m.def("add", &add, "A function that adds two numbers");
       // ...existing bindings...
    }
    ```
   - To expose a class you write yourself:
    ```cpp
    class MyClass {
    public:
       MyClass(int x) : x_(x) {}
       int get() const { return x_; }
    private:
       int x_;
    };

    PYBIND11_MODULE(cora, m) {
       py::class_<MyClass>(m, "MyClass")
          .def(py::init<int>())
          .def("get", &MyClass::get);
       // ...existing bindings...
    }
    ```
   - To expose an existing method from a C++ class (e.g., `CORA::Problem`):
    ```cpp
    #include <CORA/CORA_problem.h>
    // ...existing code...

    PYBIND11_MODULE(cora, m) {
       py::class_<CORA::Problem>(m, "Problem")
          .def("num_variables", &CORA::Problem::num_variables, "Get the number of variables");
       // ...existing bindings...
    }
    ```
    In this example, `num_variables` is a method of the `CORA::Problem` class. Adjust the method name and arguments as needed for your use case.
2. **Update `CMakeLists.txt`** if you add new source files or dependencies.
3. **Rebuild** as described above to apply changes.

## Troubleshooting
- Ensure all CORA dependencies are installed and discoverable by CMake.
- If you encounter import errors, check that the `.so` file is in your `PYTHONPATH` or current directory.
- For advanced usage, refer to the [pybind11 documentation](https://pybind11.readthedocs.io/en/stable/).

## Contact
For questions or issues, please open an issue in the main CORA repository.
