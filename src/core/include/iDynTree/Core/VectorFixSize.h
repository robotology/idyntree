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
#include <cstddef>
#include <cstring>

namespace iDynTree
{
    /**
     * Class providing a simple vector of N elements.
     *  The size of the vector is known at compile time,
     *  and it enables to avoid using dynamic memory allocation.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int VecSize> class VectorFixSize
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Array of VecSize doubles.
         */
        double m_data[VecSize];

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
        VectorFixSize(const double * in_data, const std::size_t in_size);

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /**
         * Constructor from an iDynTree::Span
         *
         * Print an error an build a vector full of zeros if in_size is not size().
         */
        VectorFixSize(iDynTree::Span<const double> vec);
#endif

        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to VectorFixSize.
         */
        ///@{
        double operator()(const std::size_t index) const;

        double& operator()(const std::size_t index);

        double operator[](const std::size_t index) const;

        double& operator[](const std::size_t index);

        double getVal(const std::size_t index) const;

        bool setVal(const std::size_t index, const double new_el);

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

        std::size_t size() const;

        ///@}

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /**
         * Copy assignment operator for spans.
         *
         * Checks that dimensions are matching through an assert.
         *
         */
        VectorFixSize & operator=(iDynTree::Span<const double> vec);
#endif

        /**
         * Raw data accessor
         *
         * @return a const pointer to a vector of size() doubles
         */
        const double * data() const;

        /**
         * Raw data accessor
         *
         * @return a pointer to a vector of size() doubles
         */
        double * data();

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
    {

    }


    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize(const double* in_data,
                                 const std::size_t in_size)
    {
        if( in_size != VecSize )
        {
            reportError("VectorFixSize","constructor","input vector does not have the right number of elements");
            this->zero();
        }
        else
        {
            memcpy(this->m_data,in_data,sizeof(double)*VecSize);
        }
    }

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000

    template<unsigned int VecSize>
    VectorFixSize<VecSize>::VectorFixSize(iDynTree::Span<const double> vec)
        : VectorFixSize<VecSize>::VectorFixSize(vec.data(), vec.size())
        {}

#endif

    template<unsigned int VecSize>
    void VectorFixSize<VecSize>::zero()
    {
        for(std::size_t i=0; i < VecSize; i++ )
        {
            this->m_data[i] = 0.0;
        }
    }


    template<unsigned int VecSize>
    double* VectorFixSize<VecSize>::data()
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::data() const
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::begin() const noexcept
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::end() const noexcept
    {
        return this->m_data + VecSize;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::cbegin() const noexcept
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    const double* VectorFixSize<VecSize>::cend() const noexcept
    {
        return this->m_data + VecSize;
    }

    template <unsigned int VecSize>
    double *VectorFixSize<VecSize>::begin() noexcept
    {
        return this->m_data;
    }

    template<unsigned int VecSize>
    double* VectorFixSize<VecSize>::end() noexcept
    {
        return this->m_data + VecSize;
    }

    template<unsigned int VecSize>
    std::size_t VectorFixSize<VecSize>::size() const
    {
        return VecSize;
    }

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
    template<unsigned int VecSize>
    VectorFixSize<VecSize> & VectorFixSize<VecSize>::operator=(iDynTree::Span<const double> vec) {
        assert(VecSize == vec.size());
        std::memcpy(this->m_data, vec.data(), VecSize*sizeof(double));
        return *this;
    }
#endif

    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::operator()(const std::size_t index) const
    {
        assert(index < VecSize);
        return this->m_data[index];
    }

    template<unsigned int VecSize>
    double & VectorFixSize<VecSize>::operator()(const std::size_t index)
    {
        assert(index < VecSize);
        return this->m_data[index];
    }

    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::operator[](const std::size_t index) const
    {
        assert(index < VecSize);
        return this->m_data[index];
    }

    template<unsigned int VecSize>
    double & VectorFixSize<VecSize>::operator[](const std::size_t index)
    {
        assert(index < VecSize);
        return this->m_data[index];
    }

    template<unsigned int VecSize>
    double VectorFixSize<VecSize>::getVal(const std::size_t index) const
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return 0.0;
        }

        return this->m_data[index];
    }

    template<unsigned int VecSize>
    bool VectorFixSize<VecSize>::setVal(const std::size_t index, const double new_el)
    {
        if( index >= this->size() )
        {
            reportError("VectorFixSize","getVal","index out of bounds");
            return false;
        }

        this->m_data[index] = new_el;

        return true;
    }

    template<unsigned int VecSize>
    void VectorFixSize<VecSize>::fillBuffer(double* buf) const
    {
        for(std::size_t i=0; i < this->size(); i++ )
        {
            buf[i] = this->m_data[i];
        }
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::toString() const
    {
        std::stringstream ss;

        for(std::size_t i=0; i < this->size(); i++ )
        {
            ss << this->m_data[i] << " ";
        }

        return ss.str();
    }

    template<unsigned int VecSize>
    std::string VectorFixSize<VecSize>::reservedToString() const
    {
        std::stringstream ss;

        for(std::size_t i=0; i < this->size(); i++ )
        {
            ss << this->m_data[i] << " ";
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
