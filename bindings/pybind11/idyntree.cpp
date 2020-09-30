#include <vector>

#include "idyntree_core.h"
// #include <iDynTree/Model/SolidShapes.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// Defines the opaque types for iDynTree bindings, see [1] for details.
// This needs to be called in the global namespace before any call to pybind11.
//
// [1]
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#making-opaque-types
// PYBIND11_MAKE_OPAQUE(std::vector<iDynTree::SolidShape*>);
// PYBIND11_MAKE_OPAQUE(std::vector<std::vector<iDynTree::SolidShape*>>);

namespace iDynTree {
namespace {

namespace py = ::pybind11;
PYBIND11_MODULE(bindings, m) {
  iDynTree::bindings::iDynTreeCoreBindings(m);

  // iDynTree::bindings::iDynTreeModelBindingsOpaqueTypes(m);
  // Binding should go in the main binding file (for whatever reason. If the
  // following line is added in iDynTreeModelBindings, we get a Python exception
  // while using the class(.
  // py::bind_vector<std::vector<SolidShape*>>(m, "LinkShapes");
//   py::bind_vector<std::vector<std::vector<SolidShape*>>>(m, "ModelShapes");
//   // And define here also the class that needs to access the opaque types.
//   py::class_<ModelSolidShapes>(m, "ModelSolidShapes")
//       .def(py::init())
//       .def_property_readonly("linkSolidShapes",
//            py::overload_cast<>(&ModelSolidShapes::getLinkSolidShapes));
//
//   iDynTree::bindings::iDynTreeModelBindings(m);
//
//   iDynTree::bindings::iDynTreeSensorsBindings(m);
//   iDynTree::bindings::iDynTreeModelIOUrdfBindings(m);
}

}  // namespace

}  // namespace iDynTree