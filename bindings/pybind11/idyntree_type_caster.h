#ifndef IDYNTREE_PYBIND11_IDYNTREE_TYPE_CASTER_H
#define IDYNTREE_PYBIND11_IDYNTREE_TYPE_CASTER_H

#include <iostream>
#include <string>
#include <type_traits>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace pybind11
{
namespace detail
{
template <template <unsigned int...> class base, typename derived>
struct is_base_of_template_int_impl
{
    template <unsigned int... Ts> static constexpr std::true_type test(const base<Ts...>*);
    static constexpr std::false_type test(...);
    using type = decltype(test(std::declval<derived*>()));
};

template <template <unsigned int...> class base, typename derived>
using is_base_of_template_int = typename is_base_of_template_int_impl<base, derived>::type;

template <typename Type>
using is_idyntree_vector_fix_size = is_base_of_template_int<iDynTree::VectorFixSize, Type>;
template <typename Type>
using is_idyntree_matrix_fix_size = is_base_of_template_int<iDynTree::MatrixFixSize, Type>;

// Type caster for all the classes that inherits from VectorDynSize
template <typename Type>
struct type_caster<Type, enable_if_t<std::is_base_of<iDynTree::VectorDynSize, Type>::value>>
{
    PYBIND11_TYPE_CASTER(Type, _(PYBIND11_TOSTRING(Type)));

    /**
     * Conversion from Python to C++
     */
    bool load(handle src, bool convert)
    {
        if (!convert && array_t<double>::check_(src))
        {
            return false;
        }

        auto buf
            = pybind11::array_t<double,
                                pybind11::array::c_style | pybind11::array::forcecast>::ensure(src);
        if (!buf)
            return false;

        // the number of dimension must be equal to 1 since it is a vector
        const auto dims = buf.ndim();
        if (dims != 1)
            return false;

        // store the data
        value = Type(buf.data(), buf.size());
        return true;
    }

    /**
     * Conversion from C++ to Python
     */
    static handle cast(Type src, return_value_policy policy, handle parent)
    {
        pybind11::array a(src.size(), src.data());
        return a.release();
    }
};

// Type caster for all the classes that inherits from MatriDynSize
template <typename Type>
struct type_caster<Type, enable_if_t<std::is_base_of<iDynTree::MatrixDynSize, Type>::value>>
{
    PYBIND11_TYPE_CASTER(Type, _(PYBIND11_TOSTRING(Type)));

    /**
     * Conversion from Python to C++
     */
    bool load(handle src, bool convert)
    {
        if (!convert && array_t<double>::check_(src))
        {
            return false;
        }

        auto buf
            = pybind11::array_t<double,
                                pybind11::array::c_style | pybind11::array::forcecast>::ensure(src);
        if (!buf)
            return false;

        // Since it is a matrix the buffer must have 2 dimensions
        const auto dims = buf.ndim();
        if (dims != 2)
            return false;

        // this can be done since numpy and iDynTree store matrices in Row-Major order
        const auto rows = buf.shape(0);
        const auto cols = buf.shape(1);
        value = Type(buf.data(), rows, cols);

        return true;
    }

    static handle cast(Type src, return_value_policy policy, handle parent)
    {
        // this can be done since numpy and iDynTree store matrices in Row-Major order
        pybind11::array a({src.rows(), src.cols()}, src.data());
        return a.release();
    }
};

// Type caster for all the classes that inherits from VectorFixSize
template <typename Type>
struct type_caster<Type, enable_if_t<is_idyntree_vector_fix_size<Type>::value>>
{
    PYBIND11_TYPE_CASTER(Type, _(PYBIND11_TOSTRING(Type)));

    /**
     * Conversion from Python to C++
     */
    bool load(handle src, bool convert)
    {
        if (!convert && array_t<double>::check_(src))
        {
            return false;
        }

        auto buf
            = pybind11::array_t<double,
                                pybind11::array::c_style | pybind11::array::forcecast>::ensure(src);
        if (!buf)
            return false;

        // It must be a vector
        const auto dims = buf.ndim();
        if (dims != 1)
            return false;

        // since the vector is fixed size it is not possible to resize it. This check if the size is
        // correct
        if (buf.size() != value.size())
            return false;

        value = Type(buf.data(), buf.size());

        return true;
    }

    /**
     * Conversion from C++ to Python
     */
    static handle cast(Type src, return_value_policy policy, handle parent)
    {
        pybind11::array a(src.size(), src.data());
        return a.release();
    }
};

// Type caster for all the classes that inherits from MatrixFixSize but that are not rotations
template <typename Type>
struct type_caster<Type,
                   enable_if_t<is_idyntree_matrix_fix_size<Type>::value
                               && !std::is_same<iDynTree::Rotation, Type>::value>>
{
    PYBIND11_TYPE_CASTER(Type, _(PYBIND11_TOSTRING(Type)));

    /**
     * Conversion from Python to C++
     */
    bool load(handle src, bool convert)
    {
        if (!convert && array_t<double>::check_(src))
        {
            return false;
        }

        auto buf
            = pybind11::array_t<double,
                                pybind11::array::c_style | pybind11::array::forcecast>::ensure(src);
        if (!buf)
            return false;

        // since it is a matrix the number of dimension must be equal to 2
        auto dims = buf.ndim();
        if (dims != 2)
            return false;

        // the matrix is fix size so it is not possible to change it's size
        if (buf.size() != value.rows() * value.cols())
            return false;

        // this can be done since numpy and iDynTree store matrices in Row-Major order
        const auto rows = buf.shape(0);
        const auto cols = buf.shape(1);
        value = Type(buf.data(), rows, cols);

        return true;
    }

    /**
     * Conversion from C++ to Python
     */
    static handle cast(Type src, return_value_policy policy, handle parent)
    {
        // this can be done since numpy and iDynTree store matrices in Row-Major order
        pybind11::array a({src.rows(), src.cols()}, src.data());
        return a.release();
    }
};

} // namespace detail
} // namespace pybind11

#endif /* end of include guard: IDYNTREE_PYBIND11_IDYNTREE_TYPE_CASTER_H */
