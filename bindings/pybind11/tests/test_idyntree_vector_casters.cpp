#include <pybind11/pybind11.h>
#include <iDynTree/pybind11/VectorCasters.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>


class TestClass
{
    iDynTree::VectorDynSize vectorDyn;
    iDynTree::VectorFixSize<3> vectorFix;
    iDynTree::MatrixDynSize matrixDyn;
    iDynTree::MatrixFixSize<4,5> matrixFix;

public:

    void setVectorDyn(const iDynTree::VectorDynSize& vector) {
        vectorDyn = vector;
    }

    const iDynTree::VectorDynSize& getVectorDyn() const {
        return vectorDyn;
    }

    void setMatrixDyn(const iDynTree::MatrixDynSize& matrix) {
        matrixDyn = matrix;
    }

    const iDynTree::MatrixDynSize& getMatrixDyn() const {
        return matrixDyn;
    }

    void setVectorFix(const iDynTree::VectorFixSize<3>& vector) {
        vectorFix = vector;
    }

    const iDynTree::VectorFixSize<3>& getVectorFix() const {
        return vectorFix;
    }

    void setMatrixFix(const iDynTree::MatrixFixSize<4,5>& matrix) {
        matrixFix = matrix;
    }

    const iDynTree::MatrixFixSize<4,5>& getMatrixFix() const {
        return matrixFix;
    }
};


namespace py = ::pybind11;
PYBIND11_MODULE(vector_casters, m) {
  py::class_<TestClass>(m, "TestClass")
    .def(py::init())
    .def_property("vector_dyn", &TestClass::getVectorDyn, &TestClass::setVectorDyn)
    .def_property("vector_fix", &TestClass::getVectorFix, &TestClass::setVectorFix)
    .def_property("matrix_dyn", &TestClass::getMatrixDyn, &TestClass::setMatrixDyn)
    .def_property("matrix_fix", &TestClass::getMatrixFix, &TestClass::setMatrixFix);
}
