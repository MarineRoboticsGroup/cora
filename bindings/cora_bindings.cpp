#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <CORA/Symbol.h>
#include <CORA/CORA_problem.h>
#include <CORA/pyfg_text_parser.h>
#include <CORA/Measurements.h>

namespace py = pybind11;

PYBIND11_MODULE(cora, m) {
    m.doc() = "Python bindings for the CORA library";

    py::class_<CORA::Symbol>(m, "Symbol")
        .def(py::init<char, int>())
        .def("chr", &CORA::Symbol::chr)
        .def("index", &CORA::Symbol::index)
        .def("__repr__",
             [](const CORA::Symbol &s) {
                 return "<cora.Symbol '" + std::string(1, s.chr()) + std::to_string(s.index()) + "'>";
             });

    py::enum_<CORA::Preconditioner>(m, "Preconditioner")
        .value("Jacobi", CORA::Preconditioner::Jacobi)
        .value("BlockCholesky", CORA::Preconditioner::BlockCholesky)
        .value("RegularizedCholesky", CORA::Preconditioner::RegularizedCholesky)
        .export_values();

    py::enum_<CORA::Formulation>(m, "Formulation")
        .value("Implicit", CORA::Formulation::Implicit)
        .value("Explicit", CORA::Formulation::Explicit)
        .export_values();

    py::class_<CORA::Problem>(m, "Problem")
        .def(py::init<int, int>(), py::arg("dim"), py::arg("relaxation_rank"))
        .def("setRank", &CORA::Problem::setRank)
        .def("setPreconditioner", &CORA::Problem::setPreconditioner)
        .def("setFormulation", &CORA::Problem::setFormulation)
        .def("updateProblemData", &CORA::Problem::updateProblemData)
        .def("dim", &CORA::Problem::dim)
        .def("addPoseVariable", py::overload_cast<const CORA::Symbol &>(&CORA::Problem::addPoseVariable))
        .def("addLandmarkVariable", py::overload_cast<const CORA::Symbol &>(&CORA::Problem::addLandmarkVariable))
        .def("addPosePrior", &CORA::Problem::addPosePrior)
        .def("addLandmarkPrior", &CORA::Problem::addLandmarkPrior)
        .def("addRelativePoseMeasurement", &CORA::Problem::addRelativePoseMeasurement)
        .def("addRelativePoseLandmarkMeasurement", &CORA::Problem::addRelativePoseLandmarkMeasurement)
        .def("addRangeMeasurement", &CORA::Problem::addRangeMeasurement)
        .def("__repr__",
             [](const CORA::Problem &p) {
                 return "<cora.Problem with " + std::to_string(p.numPoses()) + " poses and " + std::to_string(p.numLandmarks()) + " landmarks>";
             });

    py::class_<CORA::PosePrior>(m, "PosePrior")
        .def(py::init<const CORA::Symbol &, CORA::Matrix, CORA::Vector, CORA::Matrix>());

    py::class_<CORA::LandmarkPrior>(m, "LandmarkPrior")
        .def(py::init<const CORA::Symbol &, CORA::Vector, CORA::Matrix>());

    py::class_<CORA::RelativePoseMeasurement>(m, "RelativePoseMeasurement")
        .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Matrix, CORA::Vector, CORA::Matrix>());

    py::class_<CORA::RelativePoseLandmarkMeasurement>(m, "RelativePoseLandmarkMeasurement")
        .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Vector, CORA::Matrix>());

    py::class_<CORA::RangeMeasurement>(m, "RangeMeasurement")
        .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Scalar, CORA::Scalar>());

    m.def("parsePyfgTextToProblem", &CORA::parsePyfgTextToProblem, "Parse a .pyfg file to a CORA::Problem object");
}