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
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <pybind11/detail/common.h>
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

template <class DerivedSpatialVecT> void spatialVector(pybind11::module& module)
{
    py::class_<SpatialVector<DerivedSpatialVecT>>(module,
                                                  PYBIND11_TOSTRING(
                                                      PYBIND11_CONCAT(_SpatialVector,
                                                                      DerivedSpatialVecT)))
        .def_property("linear_vec3",
                      py::overload_cast<>(&SpatialVector<DerivedSpatialVecT>::getLinearVec3,
                                          py::const_),
                      &SpatialVector<DerivedSpatialVecT>::setLinearVec3)
        .def_property("angular_vec3",
                      py::overload_cast<>(&SpatialVector<DerivedSpatialVecT>::getAngularVec3,
                                          py::const_),
                      &SpatialVector<DerivedSpatialVecT>::setAngularVec3)
        .def("__getitem__",
             py::overload_cast<const unsigned int>(&SpatialVector<DerivedSpatialVecT>::operator(),
                                                   py::const_))
        .def("__setitem__",
             [](SpatialVector<DerivedSpatialVecT>& vector, std::size_t index, double new_value) {
                 vector(index) = new_value;
             })
        .def("__len__", &SpatialVector<DerivedSpatialVecT>::size)
        .def("as_vector", &SpatialVector<DerivedSpatialVecT>::asVector)
        .def("__repr__", &SpatialVector<DerivedSpatialVecT>::toString)
        .def("zero", &SpatialVector<DerivedSpatialVecT>::zero)
        .def("change_point", &SpatialVector<DerivedSpatialVecT>::changePoint)
        .def("change_coord_frame", &SpatialVector<DerivedSpatialVecT>::changeCoordFrame)
        .def(py::self + DerivedSpatialVecT())
        .def(py::self - DerivedSpatialVecT())
        .def(-py::self)
        .def_static("Zero", &SpatialVector<DerivedSpatialVecT>::Zero)
        .def_static("inverse", &SpatialVector<DerivedSpatialVecT>::inverse)
        .def_static("compose", &SpatialVector<DerivedSpatialVecT>::compose);
}

} // namespace

void iDynTreeCoreBindings(pybind11::module& module) {

  py::class_<Rotation>(module, "Rotation")
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
      .def_static("Identity", &Rotation::Identity)
      .def("__repr__", &Rotation::toString)
      .def("to_numpy", [](const Rotation& impl){
                           iDynTree::Matrix3x3 m(impl.data(), 3, 3);
                           return m;
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
      .def("__repr__", [](const SpatialInertia& inertia) {
        return inertia.asMatrix().toString();
      });

  spatialVector<SpatialMotionVector>(module);

  py::class_<SpatialMotionVector, SpatialVector<SpatialMotionVector>>(module,
                                                                      PYBIND11_TOSTRING(
                                                                          SpatialMotionVector))
      .def(py::init<>())
      .def(py::init<const LinearMotionVector3&, const AngularMotionVector3&>())
      .def(py::init<const SpatialMotionVector&>())
      .def(py::init<const SpatialVector<SpatialMotionVector> &>())
      .def(py::self * float())
      .def("as_cross_product_matrix", &SpatialMotionVector::asCrossProductMatrix)
      .def("as_cross_product_matrix_wrench", &SpatialMotionVector::asCrossProductMatrixWrench)
      .def("exp", &SpatialMotionVector::exp);

  py::class_<Twist, SpatialMotionVector>(module, "Twist")
      .def(py::init())
      .def(py::init<const LinVelocity&, const AngVelocity&>())
      .def(py::init<const SpatialMotionVector&>())
      .def(py::init<const Twist&>())
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(-py::self);

  // This is required to implicit convert:
  // - SpatialMotionVector into Twist
  // - SpatialVector<SpatialMotionVector> into SpatialMotionVector
  // without the following lines it is not possible to implicit convert iDynTree.Twist.Zero() into a
  // twist
  // https://pybind11.readthedocs.io/en/stable/advanced/classes.html#implicit-conversions
  py::implicitly_convertible<SpatialVector<SpatialMotionVector>, SpatialMotionVector>();
  py::implicitly_convertible<SpatialMotionVector, Twist>();

}
}  // namespace bindings
}  // namespace iDynTree
