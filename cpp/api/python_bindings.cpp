#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "simulator_api.hpp"

namespace py = pybind11;

PYBIND11_MODULE(trajectory_simulator_py, m) {
    m.doc() = "6-DOF Trajectory Simulator Python Bindings";

    // State structure
    py::class_<State>(m, "State")
        .def(py::init<>())
        .def_readwrite("pos_I", &State::pos_I)
        .def_readwrite("vel_I", &State::vel_I)
        .def_readwrite("quat_BI", &State::quat_BI)
        .def_readwrite("omega_B", &State::omega_B)
        .def_readwrite("mass", &State::mass);

    // ControlInput structure
    py::class_<ControlInput>(m, "ControlInput")
        .def(py::init<>())
        .def_readwrite("p", &ControlInput::p)
        .def_readwrite("q", &ControlInput::q)
        .def_readwrite("r", &ControlInput::r)
        .def_readwrite("i", &ControlInput::i);

    // VehicleParams structure
    py::class_<VehicleParams>(m, "VehicleParams")
        .def(py::init<>())
        .def_readwrite("mass", &VehicleParams::mass)
        .def_readwrite("Ixx", &VehicleParams::Ixx)
        .def_readwrite("Iyy", &VehicleParams::Iyy)
        .def_readwrite("Izz", &VehicleParams::Izz)
        .def_readwrite("Ixy", &VehicleParams::Ixy)
        .def_readwrite("Ixz", &VehicleParams::Ixz)
        .def_readwrite("Iyz", &VehicleParams::Iyz)
        .def_readwrite("ref_area", &VehicleParams::ref_area)
        .def_readwrite("ref_length", &VehicleParams::ref_length);

    // EnvironmentParams structure
    py::class_<EnvironmentParams>(m, "EnvironmentParams")
        .def(py::init<>())
        .def_readwrite("gravity", &EnvironmentParams::gravity)
        .def_readwrite("wind_I", &EnvironmentParams::wind_I)
        .def_readwrite("density", &EnvironmentParams::density)
        .def_readwrite("speed_of_sound", &EnvironmentParams::speed_of_sound);

    // AeroCoeffs structure
    py::class_<AeroCoeffs>(m, "AeroCoeffs")
        .def(py::init<>())
        .def_readwrite("Cx", &AeroCoeffs::Cx)
        .def_readwrite("Cy", &AeroCoeffs::Cy)
        .def_readwrite("Cz", &AeroCoeffs::Cz)
        .def_readwrite("Cmx", &AeroCoeffs::Cmx)
        .def_readwrite("Cmy", &AeroCoeffs::Cmy)
        .def_readwrite("Cmz", &AeroCoeffs::Cmz);

    // ThrustData structure
    py::class_<ThrustData>(m, "ThrustData")
        .def(py::init<>())
        .def_readwrite("thrust_B", &ThrustData::thrust_B)
        .def_readwrite("mass_flow_rate", &ThrustData::mass_flow_rate);

    // Main simulation step function
    m.def("step_simulation", &step_simulation,
          "Step simulation forward one time step",
          py::arg("state"),
          py::arg("control"),
          py::arg("aero_coeffs"),
          py::arg("thrust"),
          py::arg("vehicle"),
          py::arg("env"),
          py::arg("time"),
          py::arg("dt"));
}
