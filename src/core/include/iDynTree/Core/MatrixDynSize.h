/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_MATRIX_DYN_SIZE_H
#define IDYNTREE_MATRIX_DYN_SIZE_H


#include <string>

#include <iDynTree/Core/MatrixView.h>

namespace iDynTree
{
    /**
     * Class providing a simple form of matrix with dynamic size.
     *
     * \ingroup iDynTreeCore
     */
    class MatrixDynSize
    {
    private:
        /**
         * Return the raw index in the data vector of the
         * element corresponding to row and col, using row
         * major ordering.
         */
        std::size_t rawIndexRowMajor(std::size_t row, std::size_t col) const;

        /**
         * Return the raw index in the data vector of the
         * element corresponding to row and col, using col
         * major ordering.
         *
         * \warning The class stores data in row major order,
         *          this function is used just in the fillColMajorBuffer
         *          method.
         */
        std::size_t rawIndexColMajor(std::size_t row, std::size_t col) const;

        /**
         * Set the capacity of the vector, resizing the buffer pointed by m_data.
         */
        void changeCapacityAndCopyData(const std::size_t _newCapacity);

    protected:
        /**
         * Storage for the MatrixDynSize
         *
         * Pointer to an area of capacity() doubles, managed by this class.
         *
         * \warning this class stores data using the row major order
         */
        double * m_data;
        std::size_t m_rows;
        std::size_t m_cols;

        /**
         * The buffer to which m_data is pointing is m_capacity*sizeof(double).
         */
        std::size_t m_capacity;

    public:
        /**
         * Default constructor: create a 0x0 matrix.
         */
        MatrixDynSize();

        /**
         * Constructor from the rows and columns, all the element assigned to 0
         *
         * @param _rows the desired rows of the matrix.
         * @param _cols the desired cols of the matrix.
         *
         * \warning performs dynamic memory allocation operations
         */
        MatrixDynSize(std::size_t _rows, std::size_t _cols);

        /**
         * Constructor from a C-style matrix.
         *
         *
         * \warning this class stores data using the row major order
         * \warning performs dynamic memory allocation operations
         */
        MatrixDynSize(const double * in_data, const std::size_t in_rows, const std::size_t in_cols);

        /**
         * Copy constructor
         *
         * @param other the object to copy
         */
        MatrixDynSize(const MatrixDynSize& other);


        /**
         * Constructor from MatrixView object
         *
         * @param other MatrixView to copy
         * \warning performs dynamic memory allocation operations
         *
         */
        MatrixDynSize(iDynTree::MatrixView<const double> other);

        /**
         * Assignment operator
         *
         * @param other the object to copy into self
         *
         * @return *this
         */
        MatrixDynSize& operator=(const MatrixDynSize& other);

        /**
         * Assignment operator
         *
         * @param other the object to copy into self
         *
         * @return *this
         */
        MatrixDynSize& operator=(iDynTree::MatrixView<const double> other);

        /**
         * Denstructor
         *
         * \warning performs dynamic memory allocation operations
         */
        virtual ~MatrixDynSize();

        /**
         * @name Matrix interface methods.
         * Methods exposing a matrix-like interface to MatrixDynSize.
         *
         * \warning Notice that using this methods you can damage the underlyng rotation matrix.
         *          In doubt, don't use them and rely on more high level functions.
         */
        ///@{
        double operator()(const std::size_t row, const std::size_t col) const;
        double& operator()(const std::size_t row, const std::size_t col);
        double getVal(const std::size_t row, const std::size_t col) const;
        bool setVal(const std::size_t row, const std::size_t col, const double new_el);
        std::size_t rows() const;
        std::size_t cols() const;
        ///@}

        /**
         * Raw data accessor
         *
         * \warning this class stores matrix data using the row major order
         * @return a const pointer to a vector of size() doubles
         */
        const double * data() const;

        /**
         * Raw data accessor
         *
         * \warning this class stores matrix data using the row major order
         * @return a pointer to a vector of size() doubles
         */
        double * data();

        /**
         * Assign all element of the matrix to 0.
         */
        void zero();

        /**
         * Change the size of the matrix, without preserving old content.
         *
         * @param _newRows the new rows of the matrix
         * @param _newCols the new cols of the matrix
         *
         * \warning performs dynamic memory allocation operations if newRows*newCols > capacity()
         */
        void resize(const std::size_t _newRows, const std::size_t _newCols);

        /**
         * Increase the capacity of the matrix, preserving old content.
         *
         * \warning performs dynamic memory allocation operations if _newCapacity > capacity()
         */
        void reserve(const size_t _newCapacity);

        /**
         * Get the dimension (in doubles) of the buffer to which m_data is pointing.
         */
        size_t capacity() const;

        /**
         * Change the capacity of the matrix such that capacity() == rows()*cols(), preserving old content.
         *
         * \warning performs dynamic memory allocation operations if newRows*newCols != capacity()
         */
        void shrink_to_fit();

        /**
         * Assume that rowMajorBuf is pointing to
         * a buffer of rows()*cols() doubles, and fill
         * it with the content of this matrix, using
         * row major order.
         *
         * @param rowMajorBuf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillRowMajorBuffer(double * rowMajorBuf) const;

        /**
         * Assume that colMajorBuf is pointing to
         * a buffer of rows()*cols() doubles, and fill
         * it with the content of this matrix, using
         * column major order.
         *
         * @param colMajorBuf pointer to the buffer to fill
         *
         * @todo provide this for all matrix types
         *
         * \warning use this function only if you are
         *          an expert C user
         */
        void fillColMajorBuffer(double * colMajorBuf) const;


#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
        /** Typedefs to enable make_matrix_view.
         */
        ///@{
        typedef double value_type;
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

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */

