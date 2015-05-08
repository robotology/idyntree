/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MATRIX_DYN_SIZE_H
#define IDYNTREE_MATRIX_DYN_SIZE_H


#include "IMatrix.h"

#include <string>

namespace iDynTree
{
    /**
     * Class providing a simple form of matrix with dynamic size.
     */
    class MatrixDynSize: public IMatrix
    {
    private:
        /**
         * Return the raw index in the data vector of the
         * element corresponding to row and col.
         */
        const unsigned int rawIndex(int row, int col) const;

    protected:
        /**
         * Storage for the MatrixDynSize
         *
         * Pointer to an area of size() doubles, managed by this class.
         *
         * \warning this class stores data using the row major order
         */
        double * m_data;
        unsigned int m_rows;
        unsigned int m_cols;

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
        MatrixDynSize(unsigned int _rows, unsigned int _cols);

        /**
         * Constructor from a C-style matrix.
         *
         *
         * \warning this class stores data using the row major order
         * \warning performs dynamic memory allocation operations
         */
        MatrixDynSize(const double * in_data, const unsigned int in_rows, const unsigned int in_cols);

        /**
         * Denstructor
         *
         * \warning performs dynamic memory allocation operations
         */
        virtual ~MatrixDynSize();

        /**
         * @name Matrix interface methods.
         * Methods exposing a vector-like interface to RotationRaw.
         *
         * \warning Notice that using this methods you can damage the underlyng rotation matrix.
         *          In doubt, don't use them and rely on more high level functions.
         */
        ///@{
        double operator()(const unsigned int row, const unsigned int col) const;
        double& operator()(const unsigned int row, const unsigned int col);
        double getVal(const unsigned int row, const unsigned int col) const;
        bool setVal(const unsigned int row, const unsigned int col, const double new_el);
        unsigned int rows() const;
        unsigned int cols() const;
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
         * Assign all element of the vector to 0.
         */
        void zero();

        /**
         * Change the size of the vector, without preserving old content.
         *
         * @param newRows the new rows of the matrix
         * @param newCols the new cols of the matrix
         *
         * \warning performs dynamic memory allocation operations
         */
        void resize(const unsigned int _newRows, const unsigned int _newCols);


        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        ///@}

    };
}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */