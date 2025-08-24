from conan import ConanFile
from conan.tools.cmake import CMake, CMakeToolchain, CMakeDeps, cmake_layout

class CoraPlusPlus(ConanFile):
    name = "cora"
    version = "0.1.0"
    settings = "os", "compiler", "build_type", "arch"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
        "enable_vectorization": [True, False],
        "enable_openmp": [True, False],
        "enable_profiling": [True, False],
        "enable_visualization": [True, False],
        "perform_experiments": [True, False],
        "build_examples": [True, False],
        "build_python_bindings": [True, False],
        "build_tests": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "enable_vectorization": True,
        "enable_openmp": False,
        "enable_profiling": False,
        "enable_visualization": True,
        "perform_experiments": True,
        "build_examples": True,
        "build_python_bindings": True,
        "build_tests": True,
    }

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("suitesparse/7.7.0")
        self.requires("openblas/0.3.27")
        if self.options.build_tests:
            self.requires("catch2/3.6.0")
        if self.options.build_python_bindings:
            self.requires("pybind11/2.12.0")
        if self.options.enable_profiling:
            self.requires("gperftools/2.15")

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["ENABLE_VECTORIZATION"] = self.options.enable_vectorization
        tc.variables["ENABLE_OPENMP"] = self.options.enable_openmp
        tc.variables["ENABLE_PROFILING"] = self.options.enable_profiling
        tc.variables["ENABLE_VISUALIZATION"] = self.options.enable_visualization
        tc.variables["PERFORM_EXPERIMENTS"] = self.options.perform_experiments
        tc.variables["BUILD_EXAMPLES"] = self.options.build_examples
        tc.variables["BUILD_PYTHON_BINDINGS"] = self.options.build_python_bindings
        tc.variables["BUILD_TESTS"] = self.options.build_tests
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
