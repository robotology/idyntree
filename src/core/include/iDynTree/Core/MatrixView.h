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

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/Span.h>


namespace iDynTree
{
    /**
     * Class providing a simple viewer for matrices.
     *
     * \ingroup iDynTreeCore
     */

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
            if (IsRowMajor)
            {
                return (row + this->m_rows*col);
            }
            else
            {
                return (this->m_cols*row + col);
            }
        }

    public:
         template <class Container,
                  std::enable_if_t<
                        std::is_const<element_type>::value &&
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        std::is_same<typename Container::IsRowMajor, bool>::value, int> = 0>
        MatrixView(const Container& matrix)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
            static_assert(Container::IsRowMajor == IsRowMajor,
                          "[ERROR] [MatrixView] The MatrixView and the container does not use the same StoringMethod");
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_const<element_type>::value &&
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value, int> = 0>
        MatrixView(const Container& matrix)
           : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        std::is_same<typename Container::IsRowMajor, bool>::value, int> = 0>
        MatrixView(Container& matrix)
           : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
            static_assert(Container::IsRowMajor == IsRowMajor,
                          "[ERROR] [MatrixView] The MatrixView and the container does not use the same StoringMethod");
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value, int> = 0>
        MatrixView(Container& matrix)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols())
        {
        }

        MatrixView(pointer in_data, const unsigned int& in_rows, const unsigned int& in_cols)
            : m_storage(in_data), m_rows(in_rows), m_cols(in_cols)
        {
        }

        template <bool OtherIsRowMajor, std::enable_if_t<OtherIsRowMajor != IsRowMajor, int> = 0 >
        MatrixView operator=(const MatrixView<MatrixView::element_type, OtherIsRowMajor>& matrix)
        {
            assert(m_rows == matrix.rows());
            assert(m_cols == matrix.cols());

            for (unsigned int i = 0; i < m_rows; i ++)
            {
                for (unsigned int j = 0; i < m_cols; i ++)
                {
                    m_storage[this->rawIndex(i, j)] = matrix(i, j);
                }
            }

            return *this;
        }

        MatrixView operator=(const MatrixView& matrix)
        {
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

    // template deduction
    template <class Container,
              std::enable_if_t<std::is_same<typename Container::IsRowMajor, bool>::value, int> = 0>
    MatrixView(const Container& matrix) -> MatrixView<typename std::pointer_traits<decltype(std::declval<Container>().data())>::element_type, Container::IsRowMajor>;
}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */
