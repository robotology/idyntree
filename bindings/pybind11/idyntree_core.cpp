#include "idyntree_core.h"

#include <iDynTree/pybind11/VectorCasters.h>
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

template <typename VectorType>
void baseVectorDefinition(py::class_<VectorType>& vector) {
  vector
      .def("__getitem__", py::overload_cast<const std::size_t>(
                              &VectorType::operator(), py::const_))
      .def("__setitem__",
           [](VectorType& the_vector, std::size_t index,
              double new_value) { the_vector(index) = new_value; })
      .def("__len__", &VectorType::size)
      .def("__iter__",
          [](const VectorType& s) {
            return py::make_iterator(s.begin(), s.end());
          },
          py::keep_alive<
              0, 1>() /* Essential: keep object alive while iterator exists */)
      .def("set_zero", &VectorType::zero)
      .def("__repr__", &VectorType::toString)
      .def_buffer([](VectorType& v) -> py::buffer_info {
        return py::buffer_info(
            v.data(),                                /* Pointer to buffer */
            sizeof(double),                          /* Size of one scalar */
            py::format_descriptor<double>::format(), /* Python struct-style
                                                        format descriptor */
            1,                                       /* Number of dimensions */
            {v.size()},                              /* Buffer dimensions */
            {sizeof(double)} /* Strides (in bytes) for each index */
        );
      });
}

template <typename MatrixType>
void baseMatrixDefinition(py::class_<MatrixType>& matrix) {
  matrix
      .def("__getitem__",
           [](MatrixType& matrix,
              std::pair<std::size_t, std::size_t> indices) {
             return matrix.getVal(indices.first, indices.second);
           })
      .def("__setitem__",
           [](MatrixType& matrix,
              std::pair<std::size_t, std::size_t> indices, double item) {
             return matrix.setVal(indices.first, indices.second, item);
           })
      .def("rows", &MatrixType::rows)
      .def("cols", &MatrixType::cols)
      .def("set_zero", &MatrixType::zero)
      .def("__repr__", &MatrixType::toString)
      .def_buffer([](MatrixType& m) -> py::buffer_info {
        return py::buffer_info(
            m.data(),                                /* Pointer to buffer */
            sizeof(double),                          /* Size of one scalar */
            py::format_descriptor<double>::format(), /* Python struct-style
                                                        format descriptor */
            2,                                       /* Number of dimensions */
            {m.rows(), m.cols()},                    /* Buffer dimensions */
            {sizeof(double) * m.cols(), /* Strides (in bytes) for each index */
             sizeof(double)});
      });
}

void transformClassDefinition(py::class_<Transform>& transform) {
  transform.def(py::init())
      .def(py::init<const Rotation&, const Position&>())
      .def(py::init<const MatrixFixSize<4, 4>&>())
      .def_static("Identity", &Transform::Identity)
      .def_property("rotation", &Transform::getRotation, &Transform::setRotation)
      .def_property("position", &Transform::getPosition, &Transform::setPosition)
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

  // Positions, Rotations and Transforms.
  py::class_<PositionRaw> positionRaw(module, "_PositionRaw",  py::buffer_protocol());
  baseVectorDefinition(positionRaw);

  py::class_<Position, PositionRaw>(module, "Position")
      .def(py::init())
      .def(py::init<double, double, double>())
      .def(py::init([](const VectorFixSize<3>& position) {
                        return Position(position(0), position(1), position(2));
                    }))
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      .def_static("Zero", &Position::Zero);
  py::implicitly_convertible<iDynTree::VectorFixSize<3>, iDynTree::Position>();

  py::class_<RotationRaw> rotationRaw(module, "_RotationRaw", py::buffer_protocol());
  baseMatrixDefinition(rotationRaw);

  py::class_<Rotation, RotationRaw>(module, "Rotation")
      .def(py::init())
      .def(py::init<double, double, double,  //
                    double, double, double,  //
                    double, double, double>())
      .def(py::init([](const MatrixFixSize<3,3>& rotation) {
                        return Rotation(rotation);
                    }))
      .def("inverse", &Rotation::inverse)
      .def(py::self * py::self)
      .def(
          "__mul__",
          [](const Rotation& r, const Position& p) -> Position {
            return r * p;
          },
          py::is_operator())
      .def_static("Identity", &Rotation::Identity);
  py::implicitly_convertible<iDynTree::MatrixFixSize<3,3>, iDynTree::Rotation>();

  py::class_<Transform> transform(module, "Transform");
  transformClassDefinition(transform);

  // Other classes.
  py::class_<Direction> direction(module, "Direction", py::buffer_protocol());
  direction.def(py::init<double, double, double>());
  baseVectorDefinition(direction);

  py::class_<Axis>(module, "Axis")
      .def(py::init<const Direction&, const Position&>())
      .def_property("direction", &Axis::getDirection, &Axis::setDirection)
      .def_property("origin", &Axis::getOrigin, &Axis::setOrigin)
      .def("__repr__", &Axis::toString);

  py::class_<RotationalInertiaRaw> rotationalInertia(module, "RotationalInertia",
                                                     py::buffer_protocol());
  rotationalInertia.def(py::init());
  baseMatrixDefinition(rotationalInertia);

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
      .def("__repr__", [](const SpatialInertia& inertia) {
        return inertia.asMatrix().toString();
      });
}
}  // namespace bindings
}  // namespace iDynTree
