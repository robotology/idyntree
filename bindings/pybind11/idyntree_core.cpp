#include "idyntree_core.h"
#include "idyntree_type_caster.h"

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/PositionRaw.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/RotationRaw.h>
#include <iDynTree/Core/RotationalInertiaRaw.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>

namespace iDynTree {
namespace bindings {
namespace {
namespace py = ::pybind11;


void transformClassDefinition(py::class_<Transform>& transform) {
  transform.def(py::init())
      .def(py::init<const Rotation&, const Position&>())
      .def(py::init<const MatrixFixSize<4, 4>&>())
      .def_static("Identity", &Transform::Identity)
      .def_property("rotation", &Transform::getRotation, &Transform::setRotation)
      .def_property("position", &Transform::getPosition,  &Transform::setPosition)
      .def("inverse", &Transform::inverse)
      .def(py::self * py::self)
      .def(
          "__mul__",
          [](const Transform& self, const Position& pos) -> Position {
            return self * pos;
          },
          py::is_operator())
      .def(
          "__mul__",
          [](const Transform& self, const SpatialInertia& inertia)
              -> SpatialInertia { return self * inertia; },
          py::is_operator());
}
}  // namespace

void iDynTreeCoreBindings(pybind11::module& module) {

    py::class_<Rotation>(module, "Rotation")
        .def(py::init())
        .def(py::init<double,
                      double,
                      double, //
                      double,
                      double,
                      double, //
                      double,
                      double,
                      double>())
        .def("inverse", &Rotation::inverse)
        .def(py::self * py::self)
        .def(
            "__mul__",
            [](const Rotation& r, const Position& p) -> Position { return r * p; },
            py::is_operator())
        .def_static("Identity", &Rotation::Identity)
        .def("__repr__", &Rotation::toString)
        .def("to_numpy",
             [](const Rotation& impl) {
                 iDynTree::Matrix3x3 m(impl.data(), 3, 3);
                 return m;
             })
        .def("__getitem__",
             [](const Rotation& matrix, std::pair<std::size_t, std::size_t> indices) {
                 return matrix.getVal(indices.first, indices.second);
             });

    py::class_<Transform> transform(module, "Transform");
    transformClassDefinition(transform);

    // Other classes.
    py::class_<Axis>(module, "Axis")
        .def(py::init<const Direction&, const Position&>())
        .def_property("direction", &Axis::getDirection, &Axis::setDirection)
        .def_property("origin", &Axis::getOrigin, &Axis::setOrigin)
        .def("__repr__", &Axis::toString);

    py::class_<SpatialInertiaRaw>(module, "_SpatialInertiaRaw")
        .def("from_rotational_inertia_wrt_center_of_mass",
             &SpatialInertiaRaw::fromRotationalInertiaWrtCenterOfMass)
        .def("get_mass", &SpatialInertiaRaw::getMass)
        .def("get_center_of_mass", &SpatialInertiaRaw::getCenterOfMass)
        .def("get_rotational_inertia_wrt_frame_origin",
             &SpatialInertiaRaw::getRotationalInertiaWrtFrameOrigin)
        .def("get_rotational_inertia_wrt_center_of_mass",
             &SpatialInertiaRaw::getRotationalInertiaWrtCenterOfMass);

    py::class_<SpatialInertia, SpatialInertiaRaw>(module, "SpatialInertia")
        .def(py::init())
        .def(py::init<double, const PositionRaw&, const RotationalInertiaRaw&>())
        .def_static("Zero", &SpatialInertia::Zero)
        .def("as_matrix", &SpatialInertia::asMatrix)
        .def(py::self + py::self)
        .def("__repr__",
             [](const SpatialInertia& inertia) { return inertia.asMatrix().toString(); });
}
}  // namespace bindings
}  // namespace iDynTree
