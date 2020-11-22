#include <vector>

#include "idyntree_core.h"
#include "idyntree_model.h"
#include "idyntree_modelio_urdf.h"
#include "idyntree_sensors.h"

#include <pybind11/pybind11.h>


namespace iDynTree {
namespace {

namespace py = ::pybind11;
PYBIND11_MODULE(pybind, m) {
  iDynTree::bindings::iDynTreeCoreBindings(m);
  iDynTree::bindings::iDynTreeModelBindings(m);
  iDynTree::bindings::iDynTreeSensorsBindings(m);
  iDynTree::bindings::iDynTreeModelIoUrdfBindings(m);
}

}  // namespace

}  // namespace iDynTree