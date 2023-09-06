// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_DYNAMIC_SIZE_VECTOR_H
#define IDYNTREE_DYNAMIC_SIZE_VECTOR_H

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
#include <iDynTree/Span.h>
#endif

#include <string>

namespace iDynTree
{
    /**
     * Class providing a simple form of vector with dynamic size.
     * It is designed to provide seamless integration with SWIG.
     *
     * \ingroup iDynTreeCore
     */
    class VectorDynSize
    {
    protected:
        /**
         * Storage for the VectorDynSize
         *
         * Pointer to an area of capacity() doubles, managed by this class.
         */
        double * m_data;

        /**
         * Size of the vector.
         */
        std::size_t m_size;

        /**
         * The buffer to which m_data is pointing is m_capacity*sizeof(double).
         */
        std::size_t m_capacity;

        /**
         * Set the capacity of the vector, resizing the buffer pointed by m_data.
         */
        void changeCapacityAndCopyData(const std::size_t _newCapacity);

    public:
        /**
         * Default constructor: initialize the size of the array to zero.
         */
        VectorDynSize();

        /**
         * Constructor from the size, all the element assigned to 0
         *
         * @param _size the desired size of the array.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(std::size_t _size);

        /**
         * Constructor from a C-style array.
         *
         * Build
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(const double * in_data, const std::size_t in_size);

        /**
         * Copy constructor
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(const VectorDynSize& vec);

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000

        /**
         * Constructor from an iDynTree::Span
         *
         * @param vec span representing a vector
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize(iDynTree::Span<const double> vec);
#endif

        /**
         * Denstructor
         *
         * \warning performs dynamic memory allocation operations
         */
        virtual ~VectorDynSize();

        /**
         * Copy assignment operator.
         *
         * The vector will be resize to match the
         * size of the argument, and the data will
         * then be copied.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize & operator=(const VectorDynSize& vec);

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /**
         * Copy assignment operator.
         *
         * The vector will be resize to match the
         * size of the argument, and the data will
         * then be copied.
         *
         * \warning performs dynamic memory allocation operations
         */
        VectorDynSize & operator=(iDynTree::Span<const double> vec);
#endif
        /**
         * @name Vector interface methods.
         * Methods exposing a vector-like interface to PositionRaw.
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
         * Increase the capacity of the vector preserving content.
         *
         * @param newCapacity the new capacity of the vector
         * \warning performs dynamic memory allocation operations if newCapacity > capacity()
         * \note if newCapacity <= capacity(), this method does nothing and the capacity will remain unchanged.
         */
        void reserve(const std::size_t newCapacity);

        /**
         * Change the size of the vector preserving old content.
         *
         * @param newSize the new size of the vector
         * \warning performs dynamic memory allocation operations if newSize > capacity()
         */
        void resize(const std::size_t newSize);

        /**
         * Change the capacity of the vector to match the size.
         *
         * \warning performs dynamic memory allocation operations if size() != capacity()
         */
        void shrink_to_fit();

        /**
         * The buffer pointed by data() has size capacity()*sizeof(double)
         */
        size_t capacity() const;

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

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };
}

#endif /* IDYNTREE_VECTOR_DYN_SIZE_H */
