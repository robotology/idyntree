/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_MATRIX_H
#define IDYNTREE_I_MATRIX_H

namespace iDynTree
{
    /**
     * Interface (i.e. pure abstract class) exposed by  Matrix-like classes.
     *
     * \ingroup iDynTreeCore
     */
    class IMatrix
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IMatrix() = 0;

        /**
         * Return an element of the matrix by value
         * No input checking.
         *
         */
        virtual double operator()(const unsigned int row, const unsigned int col) const = 0;

        /**
         * Return a reference to a element of the matrix.
         * No input checking.
         *
         * \note This method is meant for C/C++ use only.
         */
        virtual double& operator()(const unsigned int row, const unsigned int col) = 0;

        /**
         * Return an element of the matrix by value.
         *
         * Perform boundary checking, print an error message and
         * returns 0.0 if an element outside the matrix size is requested.
         *
         */
        virtual double getVal(const unsigned int row, const unsigned int col) const = 0;

        /**
         * Set an elements of the matrix.
         *
         * Perform boundary checking, prints an error message and
         * returns false if it is requested to set an element outside
         * the matrix boundaries.
         *
         */
        virtual bool setVal(const unsigned int row, const unsigned int col, const double new_el) = 0;

        /**
         * Return the rows of the matrix.
         *
         * @return the number of rows of the matrix.
         */
        virtual unsigned int rows() const = 0;

        /**
         * Return the colums of the matrix.
         *
         * @return the number of columns of the matrix.
         */
        virtual unsigned int cols() const = 0;
    };

    /**
     * Interface (i.e. pure virtual class) for Matrix-like classes that
     * expose a pointer to their internal buffer.
     *
     * It is assume that the buffer exposed with the data() method is
     * stored using a row major ordering.
     */
    class IRawMatrix : public IMatrix
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IRawMatrix() = 0;

        /**
         * Raw data accessor
         *
         * @return a const pointer to a vector of size() doubles
         */
        virtual const double * data() const = 0;

        /**
         * Raw data accessor
         *
         * @return a pointer to a vector of size() doubles
         */
        virtual double * data() = 0;
    };

}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */