#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace iDynTree {
namespace bindings {
namespace {
namespace py = ::pybind11;

}  // namespace

void iDynTreeHighLevelBindings(pybind11::module& module) {
  py::class_<KinDynComputations>(module, "KinDynComputations")
      .def(py::init())
      .def("load_robot_model", &KinDynComputations::loadRobotModel)
      // Model inspection functions.
      .def("get_nr_of_degrees_of_freedom",
           &KinDynComputations::getNrOfDegreesOfFreedom)
      .def("get_nr_of_links", &KinDynComputations::getNrOfLinks)
      .def("get_nr_of_frames", &KinDynComputations::getNrOfFrames)
      .def("set_floating_base", &KinDynComputations::setFloatingBase)
      .def("get_floating_base", &KinDynComputations::getFloatingBase)
      // Model state functions.
      .def("set_robot_state",
           py::overload_cast<
               const iDynTree::Transform&, const iDynTree::VectorDynSize&,
               const iDynTree::Twist&, const iDynTree::VectorDynSize&,
               const iDynTree::Vector3&>(&KinDynComputations::setRobotState))
      .def("set_fixed_base_robot_state",
           py::overload_cast<const iDynTree::VectorDynSize&,
                             const iDynTree::VectorDynSize&,
                             const iDynTree::Vector3&>(
               &KinDynComputations::setRobotState))
      .def("set_joint_positions",
           py::overload_cast<const iDynTree::VectorDynSize&>(
               &KinDynComputations::setJointPos))
      //
      .def("get_frame_index", &KinDynComputations::getFrameIndex)
      .def("get_frame_name", &KinDynComputations::getFrameName)
      .def("get_world_transform", py::overload_cast<iDynTree::FrameIndex>(
                                      &KinDynComputations::getWorldTransform))
      .def("get_world_transform_for_frame_named",
           py::overload_cast<const std::string&>(
               &KinDynComputations::getWorldTransform))
      .def("get_relative_transform",
           py::overload_cast<iDynTree::FrameIndex, iDynTree::FrameIndex>(
               &KinDynComputations::getRelativeTransform))
      .def("get_relative_transform_for_frames_named",
           py::overload_cast<const std::string&, const std::string&>(
               &KinDynComputations::getRelativeTransform))
      .def("get_relative_transform_explicit",
           py::overload_cast<iDynTree::FrameIndex, iDynTree::FrameIndex,
                             iDynTree::FrameIndex, iDynTree::FrameIndex>(
               &KinDynComputations::getRelativeTransformExplicit));
}
}  // namespace bindings
}  // namespace iDynTree
