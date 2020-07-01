/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_VECTOR_FIX_SIZE_H
#define IDYNTREE_VECTOR_FIX_SIZE_H

#include <iDynTree/Core/Utils.h>
#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
#include <iDynTree/Core/Span.h>
#endif
#include <string>
#include <sstream>
#include <cassert>
#include <cstring>

#include <Eigen/Dense>

namespace iDynTree
{
    /**
     * Class providing a simple vector of N elements.
     *  The size of the vector is known at compile time,
     *  and it enables to avoid using dynamic memory allocation.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int VecSize> class VectorFixSize : public Eigen::Matrix<double, VecSize, 1>
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to 0 for perfomance reason.
         * Please initialize the data in the vector before any use.
         */
        VectorFixSize();

        /**
         * Constructor from a C-style array.
         *
         * Print an error an build a vector full of zeros if in_size is not size().
         */
        VectorFixSize(const double * in_data, const unsigned int in_size);

        /**
         * Constructor from an Eigen::MatrixBase
         *
         */
        template<class Derived>
        VectorFixSize(const Eigen::MatrixBase<Derived>& vector);

        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to VectorFixSize.
         */
        ///@{
        double getVal(const unsigned int index) const;

        bool setVal(const unsigned int index, const double new_el);

        /**
         * Returns a const iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        const double* begin() const noexcept;

        /**
         * Returns a const iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        const double* end() const noexcept;

        /**
         * Returns a const iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        const double* cbegin() const noexcept;

        /**
         * Returns a const iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        const double* cend() const noexcept;

        /**
         * Returns a iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        double* begin() noexcept;

        /**
         * Returns a iterator to the beginning of the vector
         * @note At the moment iterator is implemented as a pointer, it may change in the future.
         * For this reason it should not be used as a pointer to the data, use data() instead.
         */
        double* end() noexcept;

        unsigned int size() const;

        ///@}

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /**
         * Copy assignment operator for spans.
         *
         * Checks that dimensions are matching through an assert.
         *
         */
        VectorFixSize & operator=(const Span<const double>& vec);
#endif

        /**
         * Copy assignment operator for Eigen::MatrixBase.
         *
         */
        template <class Derived>
        VectorFixSize & operator=(const Eigen::MatrixBase<Derived>& vector);

        /**
         * Assign all element of the vector to 0.
         */
        void zero();

        /**
         * Assume that buf is pointing to
         * a buffer of size() doubles, and fill
         * it with the content of this vector.
         *
         * @param buf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillBuffer(double * buf) const;

        /**
         *  @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /** Typedefs to enable make_span.
         */
        ///@{
        typedef double value_type;

        typedef std::allocator<double> allocator_type;

        typedef typename std::allocator_traits<std::allocator<double>>::pointer pointer;

        typedef typename std::allocator_traits<std::allocator<double>>::const_pointer const_pointer;
        ///@}
#endif

    };

    //Implementation
    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize()
        : Eigen::Matrix<double, VecSize, 1>()
    {

    }

    template<unsigned int VecSize>
    template<class Derived>
    VectorFixSize<VecSize>::VectorFixSize(const Eigen::MatrixBase<Derived>& vector)
        : Eigen::Matrix<double, VecSize, 1>(vector)
    {

    }

    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize(const double* in_data,
                                 const unsigned int in_size)
        : Eigen::Matrix<double, VecSize, 1>()
    {
        if( in_size != VecSize )
        {
            reportError("VectorFixSize","constructor","input vector does not have the right number of elements");
            this->zero();
        }
        else
        {
            Eigen::Map<const Eigen::Matrix<double, VecSize, 1>> map(in_data, in_size);
            this->operator=(map);
        }
    }

    template<unsigned int VecSize>
    void VectorFixSize<VecSize>::zero()
    {
        this->setZero();
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::begin() const noexcept
    {
        return this->data();
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::end() const noexcept
    {
        return this->data() + VecSize;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::cbegin() const noexcept
    {
        return this->data();
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::cend() const noexcept
    {
        return this->data() + VecSize;
    }

    template <unsigned int VecSize>
    double *VectorFixSize<VecSize>::begin() noexcept
    {
        return this->data();
    }

    template<unsigned int VecSize>
    double* VectorFixSize<VecSize>::end() noexcept
    {
        return this->data() + VecSize;
    }

    template<unsigned int VecSize>
    unsigned int VectorFixSize<VecSize>::size() const
    {
        return VecSize;
    }

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
    template<unsigned int VecSize>
    VectorFixSize<VecSize> & VectorFixSize<VecSize>::operator=(const Span<const double>& vec) {
        assert(VecSize == vec.size());

        Eigen::Map<const Eigen::Matrix<double, VecSize, 1>> map(vec.data(), vec.size());
        this->operator=(map);
        return *this;
    }
#endif

    template<unsigned int VecSize>
    template<class Derived>
    VectorFixSize<VecSize> & VectorFixSize<VecSize>::operator=(const Eigen::MatrixBase<Derived>& vector)
    {
        Eigen::Matrix<double, VecSize, 1>::operator=(vector);
        return *this;
    }

    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::getVal(const unsigned int index) const
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return 0.0;
        }

        return this->operator[](index);
    }

    template<unsigned int VecSize>
    bool VectorFixSize<VecSize>::setVal(const unsigned int index, const double new_el)
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return false;
        }

        this->operator[](index) = new_el;

        return true;
    }

    template<unsigned int VecSize>
    void VectorFixSize<VecSize>::fillBuffer(double* buf) const
    {
        for(unsigned int i=0; i < this->size(); i++ )
        {
            buf[i] = this->operator[](i);
        }
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::toString() const
    {
        std::stringstream ss;

        for(unsigned int i=0; i < this->size(); i++ )
        {
            ss << this->operator[](i) << " ";
        }

        return ss.str();
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::reservedToString() const
    {
        std::stringstream ss;

        for(unsigned int i=0; i < this->size(); i++ )
        {
            ss << this->operator[](i) << " ";
        }

        return ss.str();
    }

    // Explicit instantiations
    // The explicit instantiations are the only ones that can be used in the API
    //  and the only ones that users are supposed to manipulate manipulate
    // Add all the explicit instantiation that can be useful, but remember to add
    // them also in the iDynTree.i SWIG file
    typedef VectorFixSize<2>  Vector2;
    typedef VectorFixSize<3>  Vector3;
    typedef VectorFixSize<4>  Vector4;
    typedef VectorFixSize<6>  Vector6;
    typedef VectorFixSize<10> Vector10;
    typedef VectorFixSize<16> Vector16;

}

#endif /* IDYNTREE_VECTOR_FIX_SIZE_H */
