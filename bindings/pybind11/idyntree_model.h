#ifndef IDYNTREE_PYBIND11_MODEL_H
#define IDYNTREE_PYBIND11_MODEL_H

#include <pybind11/pybind11.h>


namespace iDynTree {
namespace bindings {

void iDynTreeModelBindings(pybind11::module& module);

}  // namespace bindings
}  // namespace iDynTree

#endif /* end of include guard: IDYNTREE_PYBIND11_MODEL_H */