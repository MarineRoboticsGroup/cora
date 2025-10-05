#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_variable_management.h>
#include <CORA/Measurements.h>
#include <CORA/Symbol.h>
#include <CORA/pyfg_text_parser.h>
#include <Optimization/Riemannian/TNT.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(cora, m) {
  m.doc() = "Python bindings for the CORA library";

  // Optimization result status for TNT
  py::enum_<Optimization::Riemannian::TNTStatus>(m, "TNTStatus")
      .value("Gradient", Optimization::Riemannian::TNTStatus::Gradient)
      .value("PreconditionedGradient",
             Optimization::Riemannian::TNTStatus::PreconditionedGradient)
      .value("RelativeDecrease",
             Optimization::Riemannian::TNTStatus::RelativeDecrease)
      .value("Stepsize", Optimization::Riemannian::TNTStatus::Stepsize)
      .value("TrustRegion", Optimization::Riemannian::TNTStatus::TrustRegion)
      .value("IterationLimit",
             Optimization::Riemannian::TNTStatus::IterationLimit)
      .value("ElapsedTime", Optimization::Riemannian::TNTStatus::ElapsedTime)
      .value("UserFunction", Optimization::Riemannian::TNTStatus::UserFunction)
      .export_values();

  py::class_<CORA::Symbol>(m, "Symbol")
      .def(py::init<char, int>())
      .def("chr", &CORA::Symbol::chr)
      .def("index", &CORA::Symbol::index)
      .def("__repr__", [](const CORA::Symbol &s) {
        return "<cora.Symbol '" + std::string(1, s.chr()) +
               std::to_string(s.index()) + "'>";
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
      .def("addPoseVariable", py::overload_cast<const CORA::Symbol &>(
                                  &CORA::Problem::addPoseVariable))
      .def("addLandmarkVariable", py::overload_cast<const CORA::Symbol &>(
                                      &CORA::Problem::addLandmarkVariable))
      .def("addPosePrior", &CORA::Problem::addPosePrior)
      .def("addLandmarkPrior", &CORA::Problem::addLandmarkPrior)
      .def("addRelativePoseMeasurement",
           &CORA::Problem::addRelativePoseMeasurement)
      .def("addRelativePoseLandmarkMeasurement",
           &CORA::Problem::addRelativePoseLandmarkMeasurement)
      .def("addRangeMeasurement", &CORA::Problem::addRangeMeasurement)
      .def("alignEstimateToOrigin", &CORA::Problem::alignEstimateToOrigin)
      .def("getDataMatrixSize", &CORA::Problem::getDataMatrixSize)
      .def("getExpectedVariableSize", &CORA::Problem::getExpectedVariableSize)
      .def("getRelaxationRank", &CORA::Problem::getRelaxationRank)
      .def("numPoses", &CORA::Problem::numPoses)
      .def("numLandmarks", &CORA::Problem::numLandmarks)
      .def("numRangeMeasurements", &CORA::Problem::numRangeMeasurements)
      .def("numPosesDim", &CORA::Problem::numPosesDim)
      .def("projectToManifold", &CORA::Problem::projectToManifold)
      .def("__repr__", [](const CORA::Problem &p) {
        return "<cora.Problem with " + std::to_string(p.numPoses()) +
               " poses and " + std::to_string(p.numLandmarks()) + " landmarks>";
      });

  py::class_<CORA::PosePrior>(m, "PosePrior")
      .def(py::init<const CORA::Symbol &, CORA::Matrix, CORA::Vector,
                    CORA::Matrix>(),
           py::arg("id"), py::arg("R"), py::arg("t"), py::arg("cov"));

  py::class_<CORA::LandmarkPrior>(m, "LandmarkPrior")
      .def(py::init<const CORA::Symbol &, CORA::Vector, CORA::Matrix>(),
           py::arg("id"), py::arg("p"), py::arg("cov"));

  py::class_<CORA::RelativePoseMeasurement>(m, "RelativePoseMeasurement")
      .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Matrix,
                    CORA::Vector, CORA::Matrix>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("R"),
           py::arg("t"), py::arg("cov"));

  py::class_<CORA::RelativePoseLandmarkMeasurement>(
      m, "RelativePoseLandmarkMeasurement")
      .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Vector,
                    CORA::Matrix>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("t"),
           py::arg("cov"));

  py::class_<CORA::RangeMeasurement>(m, "RangeMeasurement")
      .def(py::init<const CORA::Symbol &, const CORA::Symbol &, CORA::Scalar,
                    CORA::Scalar>(),
           py::arg("first_id"), py::arg("second_id"), py::arg("r"),
           py::arg("cov"));

  m.def("parsePyfgTextToProblem", &CORA::parsePyfgTextToProblem,
        "Parse a .pyfg file to a CORA::Problem object");

  // Bind the optimization result type returned by solveCORA
  py::class_<CORA::CoraTntResult>(m, "CoraTntResult")
      .def_property_readonly("x",
                             [](const CORA::CoraTntResult &r) { return r.x; })
      .def_property_readonly("f",
                             [](const CORA::CoraTntResult &r) { return r.f; })
      .def_property_readonly(
          "elapsed_time",
          [](const CORA::CoraTntResult &r) { return r.elapsed_time; })
      .def_property_readonly(
          "objective_values",
          [](const CORA::CoraTntResult &r) { return r.objective_values; })
      .def_property_readonly(
          "time", [](const CORA::CoraTntResult &r) { return r.time; })
      .def_property_readonly(
          "iterates", [](const CORA::CoraTntResult &r) { return r.iterates; })
      .def_property_readonly(
          "gradfx_norm",
          [](const CORA::CoraTntResult &r) { return r.gradfx_norm; })
      .def_property_readonly(
          "gradient_norms",
          [](const CORA::CoraTntResult &r) { return r.gradient_norms; })
      .def_property_readonly(
          "update_step_norms",
          [](const CORA::CoraTntResult &r) { return r.update_step_norms; })
      .def_property_readonly("preconditioned_grad_f_x_norm",
                             [](const CORA::CoraTntResult &r) {
                               return r.preconditioned_grad_f_x_norm;
                             })
      .def_property_readonly(
          "status", [](const CORA::CoraTntResult &r) { return r.status; })
      .def_property_readonly("preconditioned_gradient_norms",
                             [](const CORA::CoraTntResult &r) {
                               return r.preconditioned_gradient_norms;
                             })
      .def_property_readonly(
          "inner_iterations",
          [](const CORA::CoraTntResult &r) { return r.inner_iterations; })
      .def_property_readonly(
          "update_step_M_norms",
          [](const CORA::CoraTntResult &r) { return r.update_step_M_norms; })
      .def_property_readonly(
          "gain_ratios",
          [](const CORA::CoraTntResult &r) { return r.gain_ratios; })
      .def_property_readonly(
          "trust_region_radius",
          [](const CORA::CoraTntResult &r) { return r.trust_region_radius; })
      .def("__repr__", [](const CORA::CoraTntResult &r) {
        return "<cora.CoraTntResult f=" + std::to_string(r.f) + ">";
      });

  // Expose solver and helpers
  m.def(
      "solveCORA",
      [](CORA::Problem &problem, const CORA::Matrix &x0,
         int max_relaxation_rank, bool verbose, bool log_iterates,
         bool show_iterates) {
        auto result = CORA::solveCORA(problem, x0, max_relaxation_rank, verbose,
                                      log_iterates, show_iterates);
        return result; // std::pair<CoraTntResult, std::vector<Matrix>> ->
                       // (CoraTntResult, list[np.ndarray])
      },
      py::arg("problem"), py::arg("x0"), py::arg("max_relaxation_rank") = 20,
      py::arg("verbose") = false, py::arg("log_iterates") = false,
      py::arg("show_iterates") = false,
      "Run the CORA solver. Returns (CoraTntResult, list[Matrix])");

  m.def("saddleEscape", &CORA::saddleEscape, py::arg("problem"), py::arg("Y"),
        py::arg("theta"), py::arg("v"), py::arg("gradient_tolerance"),
        py::arg("preconditioned_gradient_tolerance"),
        "Perform saddle escape step and return new iterate matrix");

  m.def("projectSolution", &CORA::projectSolution, py::arg("problem"),
        py::arg("Y"), py::arg("verbose") = false,
        "Project a solution to the correct rank and refine properties");

  // Expose core extraction helpers (rotation matrix, translation vector)
  m.def(
      "extractPose",
      [](const CORA::Problem &problem, const CORA::Matrix &Y,
         const CORA::Symbol &sym) {
        auto pr = CORA::extractPose(problem, Y, sym);
        return py::make_tuple(pr.first, pr.second);
      },
      py::arg("problem"), py::arg("Y"), py::arg("pose_sym"),
      "Extract (rotation, translation) for a pose from a solution matrix. "
      "Returns (ndarray, ndarray)");

  m.def(
      "extractPoint",
      [](const CORA::Problem &problem, const CORA::Matrix &Y,
         const CORA::Symbol &sym) {
        return CORA::extractPoint(problem, Y, sym);
      },
      py::arg("problem"), py::arg("Y"), py::arg("point_sym"),
      "Extract translation vector for a point from a solution matrix. Returns "
      "ndarray");

  // InitializationMap bindings
  py::class_<CORA::InitializationMap>(m, "InitializationMap")
      .def(py::init<>())
      .def("clear", &CORA::InitializationMap::clear)
      .def(
          "set_pose_initialization",
          [](CORA::InitializationMap &im, const CORA::Symbol &sym,
             const CORA::Matrix &R,
             const CORA::Vector &t) { im.setPoseInitialization(sym, R, t); },
          py::arg("symbol"), py::arg("R"), py::arg("t"))
      .def("has_pose_initialization",
           [](const CORA::InitializationMap &im, const CORA::Symbol &sym) {
             return im.hasPoseInitialization(sym);
           })
      .def("get_pose_rotation",
           [](const CORA::InitializationMap &im,
              const CORA::Symbol &sym) -> py::object {
             const CORA::Matrix *m = im.getPoseRotation(sym);
             if (!m)
               return py::none();
             return py::cast(*m);
           })
      .def("get_pose_translation",
           [](const CORA::InitializationMap &im,
              const CORA::Symbol &sym) -> py::object {
             const CORA::Vector *v = im.getPoseTranslation(sym);
             if (!v)
               return py::none();
             return py::cast(*v);
           })
      .def(
          "set_landmark_initialization",
          [](CORA::InitializationMap &im, const CORA::Symbol &sym,
             const CORA::Vector &p) { im.setLandmarkInitialization(sym, p); },
          py::arg("symbol"), py::arg("p"))
      .def("has_landmark_initialization",
           [](const CORA::InitializationMap &im, const CORA::Symbol &sym) {
             return im.hasLandmarkInitialization(sym);
           })
      .def("get_landmark",
           [](const CORA::InitializationMap &im,
              const CORA::Symbol &sym) -> py::object {
             const CORA::Vector *v = im.getLandmark(sym);
             if (!v)
               return py::none();
             return py::cast(*v);
           });

}