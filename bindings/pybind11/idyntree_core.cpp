#include "idyntree_core.h"

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

void vectorDynSizeClassDefinition(py::class_<VectorDynSize>& vector) {
  vector.def(py::init())
      .def(py::init<unsigned>())
      .def("__getitem__", py::overload_cast<const std::size_t>(
                              &VectorDynSize::operator(), py::const_))
      .def("__setitem__",
           [](VectorDynSize& the_vector, std::size_t index, double new_value) {
             the_vector(index) = new_value;
           })
      .def(
          "__iter__",
          [](const VectorDynSize& s) {
            return py::make_iterator(s.begin(), s.end());
          },
          py::keep_alive<
              0, 1>() /* Essential: keep object alive while iterator exists */)
      .def("__len__", &VectorDynSize::size)
      .def("set_zero", &VectorDynSize::zero)
      .def("resize", &VectorDynSize::resize)
      .def("__repr__", &VectorDynSize::toString)
      .def_buffer([](VectorDynSize& v) -> py::buffer_info {
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

template <unsigned size>
void createFixSizeVector(pybind11::module& module,
                         const std::string& class_name) {
  py::class_<VectorFixSize<size>>(module, class_name.c_str(),
                                  py::buffer_protocol())
      .def(py::init())
      .def("__getitem__", py::overload_cast<const std::size_t>(
                              &VectorFixSize<size>::operator(), py::const_))
      .def("__setitem__",
           [](VectorFixSize<size>& the_vector, std::size_t index,
              double new_value) { the_vector(index) = new_value; })
      .def("__len__", &VectorFixSize<size>::size)
      .def(
          "__iter__",
          [](const VectorFixSize<size>& s) {
            return py::make_iterator(s.begin(), s.end());
          },
          py::keep_alive<
              0, 1>() /* Essential: keep object alive while iterator exists */)
      .def("set_zero", &VectorFixSize<size>::zero)
      .def("__repr__", &VectorFixSize<size>::toString)
      .def_buffer([](VectorFixSize<size>& v) -> py::buffer_info {
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

void matrixDynSizeClassDefinition(py::class_<MatrixDynSize>& matrix) {
  matrix.def(py::init())
      .def(py::init<unsigned, unsigned>())
      .def("__getitem__",
           [](MatrixDynSize& matrix,
              std::pair<std::size_t, std::size_t> indices) {
             return matrix.getVal(indices.first, indices.second);
           })
      .def("__setitem__",
           [](MatrixDynSize& matrix,
              std::pair<std::size_t, std::size_t> indices, double item) {
             return matrix.setVal(indices.first, indices.second, item);
           })
      .def("rows", &MatrixDynSize::rows)
      .def("cols", &MatrixDynSize::cols)
      .def("set_zero", &MatrixDynSize::zero)
      .def("resize", &MatrixDynSize::resize)
      .def("__repr__", &MatrixDynSize::toString)
      .def_buffer([](MatrixDynSize& m) -> py::buffer_info {
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

template <unsigned rows, unsigned cols>
void createFixSizeMatrix(pybind11::module& module,
                         const std::string& class_name) {
  py::class_<MatrixFixSize<rows, cols>>(module, class_name.c_str(),
                                        py::buffer_protocol())
      .def(py::init())
      .def("__getitem__",
           [](MatrixFixSize<rows, cols>& matrix,
              std::pair<std::size_t, std::size_t> indices) {
             return matrix.getVal(indices.first, indices.second);
           })
      .def("__setitem__",
           [](MatrixFixSize<rows, cols>& matrix,
              std::pair<std::size_t, std::size_t> indices, double item) {
             return matrix.setVal(indices.first, indices.second, item);
           })
      .def("rows", &MatrixFixSize<rows, cols>::rows)
      .def("cols", &MatrixFixSize<rows, cols>::cols)
      .def("set_zero", &MatrixFixSize<rows, cols>::zero)
      .def("__repr__", &MatrixFixSize<rows, cols>::toString)
      .def_buffer([](MatrixFixSize<rows, cols>& m) -> py::buffer_info {
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
      .def_property_readonly("rotation", &Transform::getRotation)
      .def_property_readonly("position", &Transform::getPosition)
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
  // Vectors and matrices.
  py::class_<VectorDynSize> vector_dyn(module, "VectorDynSize",
                                       py::buffer_protocol());
  vectorDynSizeClassDefinition(vector_dyn);
  createFixSizeVector<3>(module, "Vector3");
  createFixSizeVector<4>(module, "Vector4");
  createFixSizeVector<6>(module, "Vector6");

  py::class_<MatrixDynSize> matrix_dyn(module, "MatrixDynSize",
                                       py::buffer_protocol());
  matrixDynSizeClassDefinition(matrix_dyn);
  createFixSizeMatrix<3, 3>(module, "Matrix3x3");
  createFixSizeMatrix<4, 4>(module, "Matrix4x4");
  createFixSizeMatrix<6, 6>(module, "Matrix6x6");

  // Positions, Rotations and Transforms.
  py::class_<PositionRaw, VectorFixSize<3>>(module, "_PositionRaw")
      // Do not expose constructor as we do not want users to use this class.
      .def("__repr__", &PositionRaw::toString);

  py::class_<Position, PositionRaw>(module, "Position")
      .def(py::init())
      .def(py::init<double, double, double>())
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self)
      .def_static("Zero", &Position::Zero);

  py::class_<RotationRaw, MatrixFixSize<3, 3>>(module, "_RotationRaw")
      // Do not expose constructor as we do not want users to use this class.
      .def("__repr__", &RotationRaw::toString);

  py::class_<Rotation, RotationRaw>(module, "Rotation")
      .def(py::init())
      .def(py::init<double, double, double,  //
                    double, double, double,  //
                    double, double, double>())
      .def("inverse", &Rotation::inverse)
      .def(py::self * py::self)
      .def(
          "__mul__",
          [](const Rotation& r, const Position& p) -> Position {
            return r * p;
          },
          py::is_operator())
      .def_static("Identity", &Rotation::Identity);

  py::class_<Transform> transform(module, "Transform");
  transformClassDefinition(transform);

  // Other classes.
  py::class_<Direction, VectorFixSize<3>>(module, "Direction")
      .def(py::init<double, double, double>());

  py::class_<Axis>(module, "Axis")
      .def(py::init<const Direction&, const Position&>())
      .def_property("direction", &Axis::getDirection, &Axis::setDirection)
      .def_property("origin", &Axis::getOrigin, &Axis::setOrigin)
      .def("__repr__", &Axis::toString);

  py::class_<RotationalInertiaRaw, MatrixFixSize<3, 3>>(module,
                                                        "RotationalInertia")
      .def(py::init());

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
