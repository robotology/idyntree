#ifndef IDYNTREE_PYBIND11_IDYNTREE_VECTOR_CASTERS_H
#define IDYNTREE_PYBIND11_IDYNTREE_VECTOR_CASTERS_H

#include <type_traits>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace pybind11
{
namespace detail
{

/**
 * is_vector_fixsize is used to build a type-dependent expression that check if an element
 * is an iDynTree::VectorFixSize.
 */
template<typename>
struct is_vector_fixsize : std::false_type {};

template<unsigned int N>
struct is_vector_fixsize<::iDynTree::VectorFixSize<N>> : std::true_type {};

/**
 * is_matrix_fixsize is used to build a type-dependent expression that check if an element
 * is an iDynTree::MatrixFixSize.
 */
template<typename>
struct is_matrix_fixsize : std::false_type {};

template<unsigned int nRows, unsigned int nCols>
struct is_matrix_fixsize<::iDynTree::MatrixFixSize<nRows, nCols>> : std::true_type {};

/**
 * vector_size_at_compile_time is a utility metafunction to get the compile-time size of an iDynTree
 * vector.
 */
template<typename>
struct vector_size_at_compile_time
{
    constexpr static int value = -1;
};

template<unsigned int N>
struct vector_size_at_compile_time<::iDynTree::VectorFixSize<N>>
{
    constexpr static int value = N;
};

/**
 * matrix_size_at_compile_time is a utility metafunction to get the compile-time rows and cols of
 * an iDynTree matrix.
 */
template<typename>
struct matrix_size_at_compile_time
{
    constexpr static int rows = -1;
    constexpr static int cols = -1;
};

template<unsigned int nRows, unsigned int nCols>
struct matrix_size_at_compile_time<::iDynTree::MatrixFixSize<nRows, nCols>>
{
    constexpr static int rows = nRows;
    constexpr static int cols = nCols;
};



/**
 * type_caster implementation for the iDynTree Vectors (both fixed and dynamical size)
 */
template <typename VectorType>
struct type_caster<VectorType,
                   std::enable_if_t<is_vector_fixsize<VectorType>::value
                                    || std::is_same<VectorType, iDynTree::VectorDynSize>::value>>
{
private:

    template <typename U>
    static bool checkSize(const std::size_t size, const U& value) {
        return true;
    }

    template <typename U,
              typename std::enable_if_t<is_vector_fixsize<U>::value>>
    static bool checkSize(const std::size_t size, const U& value) {
        return size == value.size();
    }

    /**
     * Python name description
     */
    static constexpr auto descriptor = _("numpy.ndarray[")
        + npy_format_descriptor<double>::name
        + _("[")
        + _<is_vector_fixsize<VectorType>::value>(_<(size_t) vector_size_at_compile_time<VectorType>::value>(),
                                             _("m"))
        + _(", 1]]");

public:
    PYBIND11_TYPE_CASTER(VectorType, descriptor);

    /**
     * Conversion from Python to C++
     */
    bool load(pybind11::handle src, bool convert) {
        namespace py = ::pybind11;

        // If we're in no-convert mode, only load if given an array of the correct type
        if (!convert && !py::isinstance<array_t<double>>(src)) {
            return false;
        }

        const auto buf = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(src);
        if (!buf) {
            return false;
	}

        // It must be a vector
        const std::size_t dims = buf.ndim();
        if (dims != 1) {
            return false;
        }

        // Check if the size of the vector is correct.
        if (!this->checkSize(buf.size(), value)) {
            return false;
        }

        // Copy the content of the python vector in to the iDynTree vector
        value = VectorType(buf.data(), buf.size());

        return true;
    }

    /**
     * Conversion from C++ to Python
     */
    static handle cast(VectorType src, pybind11::return_value_policy policy, pybind11::handle parent) {
        namespace py = ::pybind11;

        // copy the content of the iDynTree vector in the python one
        py::array a(src.size(), src.data());
        return a.release();
    }
};

/**
 * type_caster implementation for the iDynTree Matrices (both fixed and dynamical size)
 */
template <typename MatrixType>
struct type_caster<MatrixType,
                   enable_if_t<is_matrix_fixsize<MatrixType>::value
                               || std::is_same<MatrixType, iDynTree::MatrixDynSize>::value>>
{
private:

    template <typename U>
    static bool checkSize(const std::size_t size, const U& value) {
        return true;
    }

    template <typename U,
              typename std::enable_if_t<is_matrix_fixsize<U>::value>>
    static bool checkSize(const std::size_t size, const U& value) {
        return size == value.rows() * value.cols();
    }

    /**
     * Python name description
     */
    static constexpr auto descriptor = _("numpy.ndarray[")
        + npy_format_descriptor<double>::name
        + _("[")
        + _<is_matrix_fixsize<MatrixType>::value>(_<(size_t) matrix_size_at_compile_time<MatrixType>::rows>(),
                                             _("m"))
        + _(", ")
        + _<is_matrix_fixsize<MatrixType>::value>(_<(size_t) matrix_size_at_compile_time<MatrixType>::rows>(),
                                            _("n"))
        + _("]]");

public:

    PYBIND11_TYPE_CASTER(MatrixType, descriptor);

    /**
     * Conversion from Python to C++
     */
    bool load(pybind11::handle src, bool convert) {
        namespace py = ::pybind11;

        // If we're in no-convert mode, only load if given an array of the correct type
        if (!convert && !py::isinstance<array_t<double>>(src)) {
            return false;
        }

        const auto buf = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(src);
        if (!buf) {
            return false;
        }

        // since it is a matrix the number of dimension must be equal to 2
        const std::size_t dims = buf.ndim();
        if (dims != 2) {
            return false;
	}

        // Check if the size of the vector is correct.
        if (!this->checkSize(buf.size(), value)) {
            return false;
        }

        // Copy the content of the python matrix into an iDynTree Matrix
        // Here there is the assumption that the iDynTree matrices are stored using row-major
        // order
        const std::size_t rows = buf.shape(0);
        const std::size_t cols = buf.shape(1);
        value = MatrixType(buf.data(), rows, cols);

        return true;
    }

    /**
     * Conversion from C++ to Python
     */
    static handle cast(MatrixType src, pybind11::return_value_policy policy, pybind11::handle parent) {
        namespace py = ::pybind11;

        // Copy the content of the iDynTree matrix into the python one
        // this can be done since numpy and iDynTree store matrices in Row-Major order
        py::array a({src.rows(), src.cols()}, src.data());
        return a.release();
    }
};

} // namespace detail
} // namespace pybind11

#endif /* end of include guard: IDYNTREE_PYBIND11_IDYNTREE_VECTOR_CASTERS_H */
