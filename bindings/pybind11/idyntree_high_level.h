#ifndef IDYNTREE_PYBIND11_HIGH_LEVEL_H
#define IDYNTREE_PYBIND11_HIGH_LEVEL_H

#include <pybind11/pybind11.h>


namespace iDynTree {
namespace bindings {
void iDynTreeHighLevelBindings(pybind11::module& module);

}  // namespace bindings
}  // namespace iDynTree

#endif /* end of include guard: IDYNTREE_PYBIND11_HIGH_LEVEL_H */
