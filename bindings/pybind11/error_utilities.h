#ifndef IDYNTREE_PYBIND11_ERROR_UTILITIES_H
#define IDYNTREE_PYBIND11_ERROR_UTILITIES_H

#include <pybind11/pybind11.h>

namespace iDynTree {
namespace bindings {
void raisePythonException(PyObject* error_type, const char* message);
}  // namespace bindings
}  // namespace iDynTree


#endif /* end of include guard: IDYNTREE_PYBIND11_ERROR_UTILITIES_H */
