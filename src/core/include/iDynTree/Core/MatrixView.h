/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MATRIX_VIEW_H
#define IDYNTREE_MATRIX_VIEW_H

#include <string>
#include <iostream>
#include <cassert>
#include <cstring>

#include <type_traits>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/Span.h>


namespace iDynTree
{

    /**
     * is_iterable is used to build a type-dependent expression that check if an element is \a iterable
     * (i.e. the element has the methods <code>T::begin()<\code> and <code>T::end()<\code>). This
     * specific implementation is used when the the object is not iterable.
     */
    template <typename T, typename = void> struct has_is_row_major : std::false_type
    {
    };

    /**
     * is_iterable is used to build a type-dependent expression that check if an element is \a iterable
     * (i.e. the element has the methods <code>T::begin()<\code> and <code>T::end()<\code>). This
     * specific implementation is used when the the object is iterable, indeed
     * <code>void_t<\endcode> is used to detect ill-formed types in SFINAE context.
     */
    template <typename T>
    struct has_is_row_major<T, std::void_t<decltype(T::IsRowMajor)>>
        : std::true_type
    {
    };

    template<class ElementType, bool IsRowMajor = true>
    class MatrixView
    {
    public:
        using element_type = ElementType;
        using pointer = element_type*;
        using reference = element_type&;

    private:

        pointer m_storage;
        unsigned int m_rows;
        unsigned int m_cols;

        unsigned int rawIndex(int row, int col) const
        {
            if constexpr (IsRowMajor)
            {
                return (col + this->m_cols * row);
            }
            else
            {
                return (this->m_rows * col + row);
            }
        }

    public:
         template <class Container,
                  std::enable_if_t<
                        std::is_const<element_type>::value &&
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        has_is_row_major<Container>::value, int> = 0>
        MatrixView(const Container& matrix)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
            std::cerr << "costructor 1" << std::endl;
            static_assert(Container::IsRowMajor == int(IsRowMajor),
                          "[ERROR] [MatrixView] The MatrixView and the container does not use the same StoringMethod");
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_const<element_type>::value &&
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                            !has_is_row_major<Container>::value, int> = 0>
        MatrixView(const Container& matrix)
           : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
            std::cerr << "costructor 2" << std::endl;
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        has_is_row_major<Container>::value, int> = 0>
        MatrixView(Container& matrix)
           : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {

            std::cerr << "costructor 3" << std::endl;

            static_assert(Container::IsRowMajor == int(IsRowMajor),
                          "[ERROR] [MatrixView] The MatrixView and the container does not use the same StoringMethod");
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        !has_is_row_major<Container>::value, int> = 0>
        MatrixView(Container& matrix)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
            std::cerr << "costructor 4" << std::endl;
        }

        MatrixView(pointer in_data, const unsigned int& in_rows, const unsigned int& in_cols)
            : m_storage(in_data), m_rows(in_rows), m_cols(in_cols)
        {
            std::cerr << "costructor 5" << std::endl;
        }

        template <bool OtherIsRowMajor, std::enable_if_t<OtherIsRowMajor != IsRowMajor, int> = 0 >
        MatrixView& operator=(const MatrixView<element_type, OtherIsRowMajor>& matrix)
        {
            assert(m_rows == matrix.rows());
            assert(m_cols == matrix.cols());

            std::cerr << " =  operator 1" << std::endl;

            for (unsigned int i = 0; i < m_rows; i ++)
            {
                for (unsigned int j = 0; j < m_cols; j ++)
                {

                    this->m_storage[this->rawIndex(i, j)] = matrix(i, j);
                }
            }

            return *this;
        }

        MatrixView<element_type, IsRowMajor>& operator=(const MatrixView& matrix)
        {
            std::cerr << " =  operator 2" << std::endl;

            assert(m_rows == matrix.rows());
            assert(m_cols == matrix.cols());

            std::memcpy(this->m_storage, matrix.m_storage, m_rows * m_cols * sizeof(element_type));

            return *this;
        }

        pointer data() const noexcept { return m_storage; }

        /**
         * @name Matrix interface methods.
         * Methods exposing a matrix-like interface to MatrixView.
         *
         */
        ///@{
        reference operator()(const unsigned int row, const unsigned int col) const
        {
            assert(row < this->rows());
            assert(col < this->cols());
            return this->m_storage[rawIndex(row,col)];
        }

        reference getVal(const unsigned int row, const unsigned int col) const
        {
            this->operator()(row, col);
        }

        unsigned int rows() const
        {
            return this->m_rows;
        }

        unsigned int cols() const
        {
            return this->m_cols;
        }

        void zero() const
        {
            const unsigned int capacity = m_cols * m_rows;
            std::fill(m_storage, m_storage + capacity, 0.0);
        }
        ///@}
    };

    // Class Template Argument Deduction
    // This has been introduced in C++17
    // Please check here for the template deduction https://devblogs.microsoft.com/cppblog/how-to-use-class-template-argument-deduction/
    template <class Container,
              typename std::enable_if_t<has_is_row_major<Container>::value> * = nullptr>
   MatrixView(const Container& matrix) -> MatrixView<typename std::pointer_traits<decltype(std::declval<Container>().data())>::element_type, bool(Container::IsRowMajor)>;

    template <class Container,
              typename std::enable_if_t<has_is_row_major<Container>::value> * = nullptr>
   MatrixView(Container& matrix) -> MatrixView<typename std::pointer_traits<decltype(std::declval<Container>().data())>::element_type, bool(Container::IsRowMajor)>;
}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */
