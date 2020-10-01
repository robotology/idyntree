#include "error_utilities.h"

#include <pybind11/pybind11.h>

namespace iDynTree {
namespace bindings {
void raisePythonException(PyObject* error_type, const char* message) {
  PyErr_SetString(error_type, message);
  throw pybind11::error_already_set();
}
}  // namespace bindings
}  // namespace iDynTree
