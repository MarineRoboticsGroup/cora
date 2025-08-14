#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/Measurements.h>
#include <CORA/Symbol.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>

namespace py = pybind11;

// A function to solve the problem
CORA::CoraResult solve(CORA::Problem &problem, int max_rank, bool verbose,
                       bool log_iterates) {
  return CORA::solveCORA(problem, problem.getRandomInitialGuess(), max_rank,
                         verbose, log_iterates);
}

PYBIND11_MODULE(cora_py, m) {
  m.doc() = "Python bindings for the CORA library";

  py::class_<CORA::Symbol>(m, "Symbol")
      .def(py::init<unsigned char, uint64_t>())
      .def(py::init<std::string>())
      .def("chr", &CORA::Symbol::chr)
      .def("index", &CORA::Symbol::index)
      .def("__repr__", [](const CORA::Symbol &s) {
        return "<Symbol '" + s.string() + "'>";
      });

  py::class_<CORA::Problem>(m, "Problem")
      .def(py::init<int, int>(), py::arg("dim"), py::arg("rank"))
      .def("addPoseVariable", &CORA::Problem::addPoseVariable,
           py::arg("symbol"))
      .def("addLandmarkVariable", &CORA::Problem::addLandmarkVariable,
           py::arg("symbol"))
      .def("addRangeMeasurement", &CORA::Problem::addRangeMeasurement,
           py::arg("measurement"))
      .def("addRelativePoseMeasurement",
           &CORA::Problem::addRelativePoseMeasurement, py::arg("measurement"))
      .def("addRelativePoseLandmarkMeasurement",
           &CORA::Problem::addRelativePoseLandmarkMeasurement,
           py::arg("measurement"))
      .def("addPosePrior", &CORA::Problem::addPosePrior, py::arg("prior"))
      .def("addLandmarkPrior", &CORA::Problem::addLandmarkPrior,
           py::arg("prior"))
      .def("getPoseSymbolMap", &CORA::Problem::getPoseSymbolMap)
      .def("getLandmarkSymbolMap", &CORA::Problem::getLandmarkSymbolMap)
      .def("getRangeMeasurements", &CORA::Problem::getRangeMeasurements)
      .def("getRPMs", &CORA::Problem::getRPMs)
      .def("updateProblemData", &CORA::Problem::updateProblemData)
      .def("getDataMatrix", &CORA::Problem::getDataMatrix)
      .def("getTranslationExplicitSolution",
           &CORA::Problem::getTranslationExplicitSolution)
      .def("solve", &solve, py::arg("max_rank") = 20,
           py::arg("verbose") = false, py::arg("log_iterates") = false);

  py::class_<CORA::RangeMeasurement>(m, "RangeMeasurement")
      .def(py::init<CORA::Symbol, CORA::Symbol, CORA::Scalar, CORA::Scalar>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("r"),
           py::arg("cov"));

  py::class_<CORA::RelativePoseMeasurement>(m, "RelativePoseMeasurement")
      .def(py::init<CORA::Symbol, CORA::Symbol, CORA::Matrix, CORA::Vector,
                    CORA::Matrix>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("R"),
           py::arg("t"), py::arg("cov"));

  py::class_<CORA::RelativePoseLandmarkMeasurement>(
      m, "RelativePoseLandmarkMeasurement")
      .def(py::init<CORA::Symbol, CORA::Symbol, CORA::Vector, CORA::Matrix>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("t"),
           py::arg("cov"));

  py::class_<CORA::PosePrior>(m, "PosePrior")
      .def(py::init<CORA::Symbol, CORA::Matrix, CORA::Vector, CORA::Matrix>(),
           py::arg("id"), py::arg("R"), py::arg("t"), py::arg("cov"));

  py::class_<CORA::LandmarkPrior>(m, "LandmarkPrior")
      .def(py::init<CORA::Symbol, CORA::Vector, CORA::Matrix>(), py::arg("id"),
           py::arg("p"), py::arg("cov"));

  m.def("solveCORA", &CORA::solveCORA, "Solve the CORA problem");
  m.def("projectSolution", &CORA::projectSolution, "Project the solution");
}
