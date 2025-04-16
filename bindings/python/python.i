%{
#define SWIG_FILE_WITH_INIT
%}

// Convert all exceptions to RuntimeError
%include "exception.i"
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

// Include NumPy typemaps
%include "numpy.i"

// Include attributes
%include <attribute.i>

// Initialize NumPy
%init %{
import_array();
%}

// NumPy typemaps for vectors
%apply (double* IN_ARRAY1, int DIM1) {(double* in, int size)};
%apply (double** ARGOUTVIEWM_ARRAY1, int* DIM1) {(double** out, int* size)};
%apply (double* IN_ARRAY1, int DIM1) {(double* in, std::size_t size)};
%apply (double** ARGOUTVIEWM_ARRAY1, int* DIM1) {(double** out, std::size_t* size)};
%apply (double* IN_ARRAY1, int DIM1) {(double* in, std::ptrdiff_t size)};
%apply (double** ARGOUTVIEWM_ARRAY1, int* DIM1) {(double** out, std::ptrdiff_t* size)};

// NumPy typemaps for matrices
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double* in, int i, int j)};
%apply (double** ARGOUTVIEWM_ARRAY2, int* DIM1, int* DIM2) {(double** out, int* i, int* j)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double* in, std::size_t i, std::size_t j)};
%apply (double** ARGOUTVIEWM_ARRAY2, int* DIM1, int* DIM2) {(double** out, std::size_t* i, std::size_t* j)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double* in, std::ptrdiff_t i, std::ptrdiff_t j)};
%apply (double** ARGOUTVIEWM_ARRAY2, int* DIM1, int* DIM2) {(double** out, std::ptrdiff_t* i, std::ptrdiff_t* j)};

// NumPy typemaps for vectors / matrices constructors
%apply (double* IN_ARRAY1, int DIM1) {(const double* in_data, const unsigned in_size)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(const double* in_data, const unsigned in_rows, const unsigned in_cols)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* in_data, const std::size_t in_size)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(const double* in_data, const std::size_t in_rows, const std::size_t in_cols)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* in_data, const std::ptrdiff_t in_size)};
%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(const double* in_data, const std::ptrdiff_t in_rows, const std::ptrdiff_t in_cols)};

// NumPy typemaps for SpatialVector constructor
%apply (double* IN_ARRAY1, int DIM1) {(const double* linear_data, const unsigned linear_size)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* angular_data, const unsigned angular_size)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* linear_data, const std::size_t linear_size)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* angular_data, const std::size_t angular_size)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* linear_data, const std::ptrdiff_t linear_size)};
%apply (double* IN_ARRAY1, int DIM1) {(const double* angular_data, const std::ptrdiff_t angular_size)};

// NumPy typemaps for Span passed as input argument
// Custom typemap for a iDynTree::Span<const double> input, or iDynTree::Span<double> input (that is actually an output buffer)
%define SPAN_TYPEMAP(CTYPE, SPANTYPE)
  // shared “in” typemap
  %typemap(in) SPANTYPE {
    if (!PyArray_Check($input)) {
      SWIG_exception_fail(SWIG_TypeError, "Expected a numpy array.");
    }
    PyArrayObject *arr = reinterpret_cast<PyArrayObject*>($input);
    if (PyArray_NDIM(arr) != 1 || PyArray_TYPE(arr) != NPY_DOUBLE) {
      SWIG_exception_fail(SWIG_TypeError,
          "Expected a 1D numpy array of type float64.");
    }
    auto * data = static_cast<CTYPE*>(PyArray_DATA(arr));
    npy_intp n = PyArray_SIZE(arr);
    $1 = iDynTree::Span<CTYPE>(data, static_cast<size_t>(n));
  }

  // shared “typecheck” typemap
  %typemap(typecheck, precedence=SWIG_TYPECHECK_DOUBLE_PTR) SPANTYPE {
    PyArrayObject* arr = reinterpret_cast<PyArrayObject*>($input);
    $1 = ( PyArray_Check($input)
           && PyArray_NDIM(arr) == 1
           && PyArray_TYPE(arr) == NPY_DOUBLE )
         ? 1 : 0;
  }
%enddef

// Instantiate for const and non‐const
SPAN_TYPEMAP(const double,   iDynTree::Span<const double>)
SPAN_TYPEMAP(double,         iDynTree::Span<double>)

// Map the <Class>::reservedToString method to __str__ so that print(<Object>) automatically works in Python
%rename(__str__) reservedToString;

// Magic Python method for using [] on vectors
%define PYTHON_MAGIC_SET_GET_LEN_VECTOR()
    %pythoncode %{
        def __setitem__(self, index, value):
            if index >= self.size():
                raise IndexError(f"Index {index} not valid. The vector has a size of {self.size()}.")

            if not self.setVal(index, value):
                raise RuntimeError("Failed to set the value")

        def __getitem__(self, index):
            if index >= self.size():
                IndexError(f"Index {index} not valid. The vector has a size of {self.size()}.")

            return self.getVal(index)

        def __len__(self):
            return self.size()
    %}
%enddef


// Magic Python method for using [] on vectors associated to the links
%define PYTHON_MAGIC_SET_GET_LEN_LINK_ARRAY(ElementType)
     const ElementType& __getitem__(unsigned int index) {
        if (index >= self->getNrOfLinks()) {
            const std::string error = "Index " + std::to_string(index)
                                    + " not valid. The vector has a size of "
                                    + std::to_string(self->getNrOfLinks());
            throw std::out_of_range(error);
        }

        return (*self)(index);
     }

     void __setitem__(unsigned int index, const ElementType& element) {
        if (index >= self->getNrOfLinks()) {
            const std::string error = "Index " + std::to_string(index)
                                    + " not valid. The vector has a size of "
                                    + std::to_string(self->getNrOfLinks());
            throw std::out_of_range(error);
        }

        (*self)(index) = element;
     }

     %pythoncode %{
        def __len__(self):
            return self.getNrOfLinks()
     %}
%enddef


// Magic Python method for using [] on matrices
%define PYTHON_MAGIC_SET_GET_LEN_MATRIX()
    %pythoncode %{
        def __setitem__(self, indices, value):
            if not (len(indices) == 2 and indices[0] < self.rows() and indices[1] < self.cols()):
                raise IndexError(f"Indices {indices} not valid. The matrix has dimesions of ({self.rows()}, {self.cols()}).")

            if not self.setVal(indices[0], indices[1], value):
                raise RuntimeError("Failed to set the value")

        def __getitem__(self, indices):
            if not (len(indices) == 2 and indices[0] < self.rows() and indices[1] < self.cols()):
                raise IndexError(f"Indices {indices} not valid. The matrix has dimesions of ({self.rows()}, {self.cols()}).")

            return self.getVal(indices[0], indices[1])

        def __len__(self):
            return self.rows() * self.cols()
    %}
%enddef

// Shared static method for all classes inheriting from SpatialVector
%define CPP_TO_NUMPY_SPATIAL_VECTOR(SpatialClass)
    static SpatialClass FromPython(double* in, int size) {
        if (size != 6)
            throw std::runtime_error("Wrong size of input array");

        return {{in, 3}, {in + 3, 3}};
    }
%enddef

// New constructors to build objects inheriting from SpatialVector with two 1x3 arrays
// and one 1x6 array
%define CPP_SPATIAL_CLASS_LINANG_CONSTRUCTOR(SpatialClass)
    SpatialClass(const double* linear_data, const unsigned linear_size,
                 const double* angular_data, const unsigned angular_size)
    {
        if (linear_size != 3)
            throw std::runtime_error("Wrong size of linear component");

        if (angular_size != 3)
            throw std::runtime_error("Wrong size of angular component");

        return new iDynTree::SpatialClass({linear_data, linear_size},
                                          {angular_data, angular_size});
    }

    SpatialClass(const double* in_data, const unsigned in_size)
    {
        if (in_size != 6)
            throw std::runtime_error("Wrong size of input spatial vector");

        return new iDynTree::SpatialClass({in_data, 3},
                                          {in_data + 3, 3});
    }
%enddef

//
// VectorDynSize
//
%extend iDynTree::VectorDynSize {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    void toNumPy(double** out, int* size) {
        *out = static_cast<double*>(malloc(self->size() * sizeof(double)));
        std::copy(self->begin(), self->end(), *out);
        *size = self->size();
    }

    static iDynTree::VectorDynSize FromPython(double* in, int size) {
        return {in, static_cast<unsigned>(size)};
    }
};

//
// MatrixDynSize
//
%extend iDynTree::MatrixDynSize {

    PYTHON_MAGIC_SET_GET_LEN_MATRIX()

    void toNumPy(double** out, int* i, int* j) {
        const unsigned size = self->rows() * self->cols();
        *out = static_cast<double*>(malloc(size * sizeof(double)));
        std::copy(self->data(), self->data() + size, *out);
        *i = self->rows();
        *j = self->cols();
    }

    static iDynTree::MatrixDynSize FromPython(double* in, int i, int j) {
        return {in, static_cast<unsigned>(i), static_cast<unsigned>(j)};
    }
};

//
// VectorFixSize
//
%extend iDynTree::VectorFixSize {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    void toNumPy(double** out, int* size) {
        *out = static_cast<double*>(malloc(self->size() * sizeof(double)));
        std::copy(self->begin(), self->end(), *out);
        *size = self->size();
    }

    static iDynTree::VectorFixSize<VecSize> FromPython(double* in, int size) {
        if (size != VecSize)
            throw std::runtime_error("Wrong size of input array");
        return {in, static_cast<unsigned>(size)};
    }
};

//
// MatrixFixSize
//
%extend iDynTree::MatrixFixSize {

    PYTHON_MAGIC_SET_GET_LEN_MATRIX()

    void toNumPy(double** out, int* i, int* j) {
        const unsigned size = self->rows() * self->cols();
        *out = static_cast<double*>(malloc(size * sizeof(double)));
        std::copy(self->data(), self->data() + size, *out);
        *i = self->rows();
        *j = self->cols();
    }

    static iDynTree::MatrixFixSize<nRows, nCols> FromPython(double* in, int i, int j) {
        if (i != nRows || j != nCols)
            throw std::runtime_error("Wrong size of input matrix");

        return {in, static_cast<unsigned>(i), static_cast<unsigned>(j)};
    }
};

//
// JointPosDoubleArray
//
%extend iDynTree::JointPosDoubleArray {
    static iDynTree::JointPosDoubleArray FromPython(double* in, int size) {
        iDynTree::JointPosDoubleArray jntPos;
        jntPos.resize(static_cast<unsigned>(size));
        for (int i = 0; i < size; i++) {
            jntPos(static_cast<unsigned>(i)) = in[i];
        }
        return jntPos;
    }

    JointPosDoubleArray(const double* in_data, const unsigned in_size) {
        iDynTree::JointPosDoubleArray* jntPos = new iDynTree::JointPosDoubleArray();
        jntPos->resize(in_size);
        for (unsigned i = 0; i < in_size; i++) {
            (*jntPos)(i) = in_data[i];
        }
        return jntPos;
    }
}

//
// JointDOFsDoubleArray
//
%extend iDynTree::JointDOFsDoubleArray {
    static iDynTree::JointDOFsDoubleArray FromPython(double* in, int size) {
        iDynTree::JointDOFsDoubleArray jntDOFs;
        jntDOFs.resize(static_cast<unsigned>(size));
        for (int i = 0; i < size; i++) {
            jntDOFs(static_cast<unsigned>(i)) = in[i];
        }
        return jntDOFs;
    }

    JointDOFsDoubleArray(const double* in_data, const unsigned in_size) {
        iDynTree::JointDOFsDoubleArray* jntDOFs = new iDynTree::JointDOFsDoubleArray();
        jntDOFs->resize(in_size);
        for (unsigned i = 0; i < in_size; i++) {
            (*jntDOFs)(i) = in_data[i];
        }
        return jntDOFs;
    }
}

//
// SpatialVector
//
%extend iDynTree::SpatialVector {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    void toNumPy(double** out, int* size) {
        *out = static_cast<double*>(malloc(self->size() * sizeof(double)));
        std::copy(self->getLinearVec3().begin(), self->getLinearVec3().end(), *out);
        std::copy(self->getAngularVec3().begin(), self->getAngularVec3().end(), *out + 3);
        *size = self->size();
    }
};

//
// Wrench
//
%extend iDynTree::Wrench {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    CPP_SPATIAL_CLASS_LINANG_CONSTRUCTOR(Wrench);
    CPP_TO_NUMPY_SPATIAL_VECTOR(iDynTree::Wrench)

};

//
// Twist
//
%extend iDynTree::Twist {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    CPP_SPATIAL_CLASS_LINANG_CONSTRUCTOR(Twist);
    CPP_TO_NUMPY_SPATIAL_VECTOR(iDynTree::Twist)

};

//
// SpatialAcc
//
%extend iDynTree::SpatialAcc {

    PYTHON_MAGIC_SET_GET_LEN_VECTOR()

    CPP_SPATIAL_CLASS_LINANG_CONSTRUCTOR(SpatialAcc);
    CPP_TO_NUMPY_SPATIAL_VECTOR(iDynTree::SpatialAcc)

};

//
// Vector associated to the links
//
%extend iDynTree::LinkPositions {
    PYTHON_MAGIC_SET_GET_LEN_LINK_ARRAY(iDynTree::Transform)
};

%extend iDynTree::LinkVelArray {
    PYTHON_MAGIC_SET_GET_LEN_LINK_ARRAY(iDynTree::Twist)
};

%extend iDynTree::LinkAccArray {
    PYTHON_MAGIC_SET_GET_LEN_LINK_ARRAY(iDynTree::SpatialAcc)
};

%extend iDynTree::LinkWrenches {
    PYTHON_MAGIC_SET_GET_LEN_LINK_ARRAY(iDynTree::Wrench)
};

%extend iDynTree::Rotation {
    static iDynTree::Rotation FromPython(double* in, int i, int j) {
        if (i != 3 || j != 3)
            throw std::runtime_error("Wrong size of input matrix");

        return iDynTree::Rotation(in, static_cast<unsigned>(i), static_cast<unsigned>(j));
    }

    Rotation(const double* in_data, const std::ptrdiff_t in_rows, const std::ptrdiff_t in_cols) {
        iDynTree::Rotation* rot = new iDynTree::Rotation(in_data,
                                                         static_cast<unsigned>(in_rows),
                                                         static_cast<unsigned>(in_cols));
        return rot;
    }
};

%extend iDynTree::Position {
    static iDynTree::Position FromPython(double* in, int size) {
        if (size != 3)
            throw std::runtime_error("Wrong size of input array");

        return iDynTree::Position(in, static_cast<unsigned>(size));
    }

    Position(const double* in_data, const unsigned in_size) {
        iDynTree::Position* pos = new iDynTree::Position(in_data, in_size);
        return pos;
    }
};

%extend iDynTree::Transform {

    void setPosition(const double* in_data, const unsigned in_size) {
        $self->setPosition(iDynTree::Position(in_data, static_cast<unsigned>(in_size)));
    }

    void setRotation(const double* in_data,
                     const std::ptrdiff_t in_rows,
                     const std::ptrdiff_t in_cols) {
        $self->setRotation(iDynTree::Rotation(in_data,
                                              static_cast<unsigned>(in_rows),
                                              static_cast<unsigned>(in_cols)));
    }
};


%attributeref(iDynTree::FreeFloatingPos, iDynTree::Transform&, worldBasePos)
%attributeref(iDynTree::FreeFloatingPos, iDynTree::JointPosDoubleArray&, jointPos)
%attributeref(iDynTree::FreeFloatingVel, iDynTree::Twist&, baseVel)
%attributeref(iDynTree::FreeFloatingVel, iDynTree::JointDOFsDoubleArray&, jointVel)
%attributeref(iDynTree::FreeFloatingAcc, iDynTree::SpatialAcc&, baseAcc)
%attributeref(iDynTree::FreeFloatingAcc, iDynTree::JointDOFsDoubleArray&, jointAcc)
