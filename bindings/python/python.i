%rename(__str__) reservedToString;
%include "python/exception.i"

%{
#define SWIG_FILE_WITH_INIT
%}

%include "numpy.i"

%init %{
import_array();
%}

%{
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
%}


%exception {
  try {
    $function
  } catch (const std::invalid_argument& e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}

%inline %{
void size_of_vector(PyObject* inputObj, size_t *out_size) {
  // Is it a Numpy array?
  if (!PyArray_Check(inputObj)) {
      // Check if it is a Python List
      if (!PyList_Check(inputObj)) {
          throw std::invalid_argument("Parameter is not a list or a NumPy array");
      }
      Py_ssize_t size = PyList_Size(inputObj);
      *out_size = size;
  } else {
      PyArrayObject *array = (PyArrayObject*)inputObj;
      // This is a Numpy array
      if (PyArray_NDIM(array) > 1) {
          throw std::invalid_argument("iDynTree vector supports only one-dimensional data");
      }
      npy_intp size = PyArray_SIZE(array);
      *out_size = size;
    }
}

void size_of_matrix(PyObject* arg, size_t *out_rows, size_t* out_cols)
{
    // Is it a Numpy array?
    if (!PyArray_Check(arg)) {
        // Check if it is a Python List
        if (!PyList_Check(arg)) {
            throw std::invalid_argument("Parameter is not a list or a NumPy array");
        }
        Py_ssize_t rows = PyList_Size(arg);
        // [[1.0, 2.0], [3, 4], [5, 6]]
        // I know it is slow, but we first iterate so as to undertand the
        // size of the matrix
        int cols = -1;
        for (Py_ssize_t c = 0; c < rows; ++c) {
            // Iterate on all the sub lists
            // We have to check that: the size are the same
            PyObject* column = PyList_GetItem(arg, c);
            if (!PyList_Check(column)) {
                throw std::invalid_argument("Malformed matrix");
            }
            // Check the size: this will be the column
            Py_ssize_t colSize = PyList_Size(column);
            if (cols < 0) {
                cols = colSize;
            }
            if (colSize != cols) {
                throw std::invalid_argument("List has varying size columns");
            }
        }
        // We now have the size.
        *out_rows = rows;
        *out_cols = cols;
        return;
    }
    PyArrayObject *array = (PyArrayObject*)arg;
    // This is a Numpy array
    if (PyArray_NDIM(array) != 2) {
        throw std::invalid_argument("iDynTree matrices supports only two-dimensional arrays");
    }
    npy_intp *dimensions = PyArray_DIMS(array);
    *out_rows = dimensions[0];
    *out_cols = dimensions[1];
}
%}

namespace iDynTree
{

%define VECTOR_TONUMPY(vec)
    npy_intp size[1]; // single dimension
    size[0] = vec.size();
    PyObject* newNumPy  = PyArray_SimpleNew(1, size, NPY_DOUBLE);
    memcpy(PyArray_DATA((PyArrayObject*)newNumPy), vec.data(), sizeof(double) * size[0]);
    return newNumPy;
%enddef

%define VECTOR_ACCESSORS()
double __getitem__(int i) {
    return (*self).getVal(i);
}

void __setitem__(int i, double val) {
    (*self).setVal(i, val);
}

%enddef

%define MATRIX_TONUMPY()
PyObject* toNumPy() {
    npy_intp size[2]; // two dimensions
    size[0] = (*self).rows();
    size[1] = (*self).cols();
    // Should be C-style by default
    PyObject* newNumPy = PyArray_SimpleNew(2, size, NPY_DOUBLE);
    memcpy(PyArray_DATA((PyArrayObject*)newNumPy), (*self).data(), sizeof(double) * size[0] * size[1]);
    return newNumPy;
}
%enddef

%define VECTOR_FROMPYTHON(inputObj, destination, fixed)
      size_t _v_size;
      size_of_vector(inputObj, &_v_size);
      // Is it a Numpy array?
      if (!PyArray_Check(inputObj)) {
#if fixed
          size_t currentSize = destination.size();
          if (currentSize != _v_size) {
              throw std::invalid_argument("Mismatched dimension for fixed size array. Got " + std::to_string(_v_size) + ", expected " + std::to_string(currentSize));
          }
#else
          destination.resize(_v_size);
#endif
          for (Py_ssize_t idx = 0; idx < _v_size; ++idx) {
              PyObject* object = PyList_GetItem(inputObj, idx);
              PyErr_Clear();
              double value = PyFloat_AsDouble(object);
              if (value == -1) {
                  PyObject *conversionError = PyErr_Occurred();
                  if (conversionError) {
                      throw std::invalid_argument("Element at index " + std::to_string(idx) + " cannot be converted to double");
                  }
              }
              destination.setVal(idx, value);
          }
      } else {
          PyArrayObject *array = (PyArrayObject*)inputObj;
          void * data = PyArray_DATA(array);
#if fixed
          size_t currentSize = destination.size();
          if (currentSize != _v_size) {
              throw std::invalid_argument("Mismatched dimension for fixed size array. Got " + std::to_string(_v_size) + ", expected " + std::to_string(currentSize));
          }
#else
          destination.resize(_v_size);
#endif
          memcpy(destination.data(), data, sizeof(double) * _v_size);
      }
%enddef

%define MATRIX_FROMPYTHON(arg, destination, fixed)
    size_t rows, cols;
    size_of_matrix(arg, &rows, &cols);
    // Is it a Numpy array?
    if (!PyArray_Check(arg)) {
#if fixed
        if (rows != destination.rows() || cols != destination.cols()) {
            throw std::invalid_argument("Mismatched dimension for fixed size matrix. Got " + std::to_string(rows) + "x" + std::to_string(cols) + ", expected " + std::to_string(destination.rows()) + "x" + std::to_string(destination.cols()));
        }
#else
        destination.resize(rows, cols);
#endif
        for (Py_ssize_t r = 0; r < rows; ++r) {
            PyObject* innerList = PyList_GetItem(arg, r);
            for (Py_ssize_t c = 0; c < cols; ++c) {
                PyObject* object = PyList_GetItem(innerList, c);
                PyErr_Clear();
                double value = PyFloat_AsDouble(object);
                if (value == -1) {
                    PyObject *conversionError = PyErr_Occurred();
                    if (conversionError) {
                        throw std::invalid_argument("Element at index (" + std::to_string(r) + "," + std::to_string(c) +") cannot be converted to double");
                    }
                }
                destination.setVal(r, c, value);
            }
        }
    }
    else {
        PyArrayObject *array = (PyArrayObject*)arg;
#if fixed
        size_t current_rows = destination.rows();
        size_t current_cols = destination.cols();
        if (rows != current_rows || cols != current_cols) {
          throw std::invalid_argument("Mismatched dimension for fixed size matrix. Got " + std::to_string(rows) + "x" + std::to_string(cols) + ", expected " + std::to_string(current_rows) + "x" + std::to_string(current_cols));
        }
#else
        destination.resize(rows, cols);
#endif
        void * data = PyArray_DATA(array);
        memcpy(destination.data(), data, sizeof(double) * rows * cols);
    }
%enddef

%extend VectorDynSize {
    PyObject* toNumPy() {
        VECTOR_TONUMPY((*self))
    }

    VECTOR_ACCESSORS()

    size_t __len__() {
        return (*self).size();
    }

%define DYN_SIZE_VECTOR_CREATE()
    size_t size;
    size_of_vector(arg, &size);
    iDynTree::VectorDynSize *v = new iDynTree::VectorDynSize(size);
    VECTOR_FROMPYTHON(arg, (*v), 0)
    return v;
%enddef

    VectorDynSize(PyObject *arg) {
        DYN_SIZE_VECTOR_CREATE()
    }

    static VectorDynSize* FromPython(PyObject *arg) {
        DYN_SIZE_VECTOR_CREATE()
    }
};

%extend VectorFixSize {
    PyObject* toNumPy() {
        VECTOR_TONUMPY((*self))
    }

    VECTOR_ACCESSORS()

    size_t __len__() {
        return VecSize;
    }

%define FIX_SIZE_VECTOR_CREATE()
    size_t size;
    size_of_vector(arg, &size);
    if (size != VecSize) {
        throw std::invalid_argument("Size mismatch: Got " + std::to_string(size) + ". Expected " + std::to_string(VecSize));
    }
    iDynTree::VectorFixSize<VecSize> *v = new iDynTree::VectorFixSize<VecSize>();
    VECTOR_FROMPYTHON(arg, (*v), 1)
    return v;
%enddef

    static VectorFixSize<VecSize>* FromPython(PyObject *arg) {
        FIX_SIZE_VECTOR_CREATE()
    }

    VectorFixSize<VecSize>(PyObject *arg) {
        FIX_SIZE_VECTOR_CREATE()
    }

};

%extend MatrixDynSize {
    MATRIX_TONUMPY()

%define DYN_SIZE_MATRIX_CREATE()
    size_t _m_rows, _m_cols;
    size_of_matrix(arg, &_m_rows, &_m_cols);
    iDynTree::MatrixDynSize *m = new iDynTree::MatrixDynSize(_m_rows, _m_cols);
    MATRIX_FROMPYTHON(arg, (*m), 0)
    return m;
%enddef

    static MatrixDynSize* FromPython(PyObject *arg) {
        DYN_SIZE_MATRIX_CREATE()
    }

    MatrixDynSize(PyObject *arg) {
        DYN_SIZE_MATRIX_CREATE()
    }
};

%extend MatrixFixSize {
    MATRIX_TONUMPY()

%define FIX_SIZE_MATRIX_CREATE()
    size_t _m_rows, _m_cols;
    size_of_matrix(arg, &_m_rows, &_m_cols);
    if (_m_rows != nRows || _m_cols != nCols) {
        throw std::invalid_argument("Size mismatch: Got " + std::to_string(_m_rows) + "x" + std::to_string(_m_cols) +
        ". Expected " + std::to_string(nRows) + "x" + std::to_string(nCols));
    }
    iDynTree::MatrixFixSize<nRows, nCols> *m = new iDynTree::MatrixFixSize<nRows, nCols>();
    MATRIX_FROMPYTHON(arg, (*m), 1)
    return m;
%enddef

    static MatrixFixSize<nRows, nCols>* FromPython(PyObject *arg) {
        FIX_SIZE_MATRIX_CREATE()
    }

    MatrixFixSize<nRows, nCols>(PyObject *arg) {
        FIX_SIZE_MATRIX_CREATE()
    }
};

%define SPATIAL_VECTOR_CREATE(destination)
    iDynTree::Vector6 pythonObject;
    VECTOR_FROMPYTHON(arg, pythonObject, 1)
    iDynTree::Vector3 force;
    memcpy(force.data(), pythonObject.data(), sizeof(double) * 3);
    iDynTree::Vector3 torque;
    memcpy(torque.data(), pythonObject.data() + 3, sizeof(double) * 3);
    destination.setLinearVec3(force);
    destination.setAngularVec3(torque);
%enddef

%extend Wrench {
    PyObject* toNumPy() {
        iDynTree::Vector6 vec = (*self).asVector();
        VECTOR_TONUMPY(vec)
    }

    static Wrench* FromPython(PyObject *arg) {
        iDynTree::Wrench *newWrench = new iDynTree::Wrench();
        SPATIAL_VECTOR_CREATE((*newWrench))
        return newWrench;
    }

    Wrench(PyObject *arg) {
        iDynTree::Wrench *newWrench = new iDynTree::Wrench();
        SPATIAL_VECTOR_CREATE((*newWrench))
        return newWrench;
    }

    VECTOR_ACCESSORS()

    size_t __len__() {
        return 6;
    }

}

%extend Twist {
    PyObject* toNumPy() {
        iDynTree::Vector6 vec = (*self).asVector();
        VECTOR_TONUMPY(vec)
    }

    static Twist* FromPython(PyObject *arg) {
        iDynTree::Twist *newTwist = new iDynTree::Twist();
        SPATIAL_VECTOR_CREATE((*newTwist))
        return newTwist;
    }

    Twist(PyObject *arg) {
        iDynTree::Twist *newTwist = new iDynTree::Twist();
        SPATIAL_VECTOR_CREATE((*newTwist))
        return newTwist;
    }

    VECTOR_ACCESSORS()

    size_t __len__() {
        return 6;
    }
}

%extend SpatialAcc {
    PyObject* toNumPy() {
        iDynTree::Vector6 vec = (*self).asVector();
        VECTOR_TONUMPY(vec)
    }

    static SpatialAcc* FromPython(PyObject *arg) {
        iDynTree::SpatialAcc *newAcc = new iDynTree::SpatialAcc();
        SPATIAL_VECTOR_CREATE((*newAcc))
        return newAcc;
    }

    SpatialAcc(PyObject *arg) {
        iDynTree::SpatialAcc *newAcc = new iDynTree::SpatialAcc();
        SPATIAL_VECTOR_CREATE((*newAcc))
        return newAcc;
    }

    VECTOR_ACCESSORS()

    size_t __len__() {
        return 6;
    }
}

}


%pythoncode %{
#these need to be called after iDynTree module has been loaded
def init_helpers():
    import warnings
    def _fromList_deprecated_wrapper(cls, list):
        warnings.warn("'fromList' is deprecated. Please use 'fromPython'.", FutureWarning)
        return cls.FromPython(list)

    VectorDynSize.fromList = classmethod(_fromList_deprecated_wrapper)

    LinearForceVector3.fromList = classmethod(_fromList_deprecated_wrapper)
    LinearMotionVector3.fromList = classmethod(_fromList_deprecated_wrapper)
    AngularForceVector3.fromList = classmethod(_fromList_deprecated_wrapper)
    AngularMotionVector3.fromList = classmethod(_fromList_deprecated_wrapper)

    SpatialAcc.fromList = classmethod(_fromList_deprecated_wrapper)
    ClassicalAcc.fromList = classmethod(_fromList_deprecated_wrapper)
    SpatialInertia.fromList = classmethod(_fromList_deprecated_wrapper)
    SpatialMomentum.fromList = classmethod(_fromList_deprecated_wrapper)
    SpatialMotionVector.fromList = classmethod(_fromList_deprecated_wrapper)
    Twist.fromList = classmethod(_fromList_deprecated_wrapper)
    Wrench.fromList = classmethod(_fromList_deprecated_wrapper)

    warnings.warn("init_helpers is deprecated. Please use 'fromPython' instead of 'fromList' and remove the call to 'init_helpers'.", FutureWarning)

def init_numpy_helpers():
    import warnings
    warnings.warn("init_numpy_helpers is deprecated. Support for toNumPy is now enabled by default. Please, remove the call to 'init_numpy_helpers'.", FutureWarning)
%}
