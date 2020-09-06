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

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/Span.h>


namespace iDynTree
{
    /**
     * Class providing a simple viewer for matrices.
     *
     * \ingroup iDynTreeCore
     */

    template<class ElementType>
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
        bool m_isRowMajor{true};

        unsigned int rawIndex(int row, int col) const
        {
            if (m_isRowMajor)
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
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols(), Container::IsRowMajor)
        {
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_const<element_type>::value &&
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value, int> = 0>
        MatrixView(const Container& matrix, const bool& isRowMajor = true)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols(), isRowMajor)
        {
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value &&
                        std::is_same<typename Container::IsRowMajor, bool>::value, int> = 0>
        MatrixView(Container& matrix) : MatrixView(matrix.data(), matrix.rows(), matrix.cols(), Container::IsRowMajor)
        {
        }

        template <class Container,
                  std::enable_if_t<
                        std::is_convertible<decltype(std::declval<Container>().data()),
                                            pointer>::value, int> = 0>
        MatrixView(Container& matrix, const bool& isRowMajor = true)
            : MatrixView(matrix.data(), matrix.rows(), matrix.cols(), isRowMajor)
        {
        }

        MatrixView(pointer in_data, const unsigned int in_rows, const unsigned int in_cols, const bool& isRowMajor)
            : m_storage(in_data), m_rows(in_rows), m_cols(in_cols), m_isRowMajor(isRowMajor)
        {
        }

        MatrixView& operator=(const MatrixView& matrix)
        {
            if(matrix == *this)
                return *this;

            assert(m_rows == matrix.rows());
            assert(m_cols == matrix.cols());

            for (unsigned int i = 0; i < m_rows; i ++)
            {
                for (unsigned int j = 0; i < m_cols; i ++)
                {
                    m_storage[this->rawIndex(i, j)] = matrix(i, j);
                }
            }

            m_storage = matrix.data();

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
        ///@}
    };

}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */
