/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro, Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Triplets.h"
#include "MatrixDynSize.h"
#include "MatrixFixSize.h"
#include "SparseMatrix.h"

#include <algorithm>

namespace iDynTree {

    Triplet::Triplet(unsigned _row, unsigned _column, double _value)
    : row(_row)
    , column(_column)
    , value(_value) {}

    bool Triplet::operator<(const iDynTree::Triplet &other) const
    {
        return rowMajorCompare(*this, other);
    }

    bool Triplet::operator==(const iDynTree::Triplet &other) const
    {
        return row == other.row && column == other.column;
    }

    bool Triplet::rowMajorCompare(const iDynTree::Triplet &a, const iDynTree::Triplet &b)
    {
        return a.row < b.row
        || (a.row == b.row && a.column < b.column);
    }

    bool Triplet::columnMajorCompare(const iDynTree::Triplet &a, const iDynTree::Triplet &b)
    {
        return a.column < b.column
        || (a.column == b.column && a.row < b.row);
    }

    void Triplets::reserve(unsigned size)
    {
        m_triplets.reserve(size);
    }

    void Triplets::clear()
    {
        m_triplets.clear();
    }

    void Triplets::addSubMatrix(unsigned startingRow,
                                unsigned startingColumn,
                                const MatrixDynSize &matrix)
    {
        unsigned rows = matrix.rows();
        unsigned cols = matrix.cols();

        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (unsigned row = 0; row < rows; ++row) {
            for (unsigned col = 0; col < cols; ++col) {
                m_triplets.push_back(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void Triplets::addSubMatrix(unsigned startingRow,
                                unsigned startingColumn,
                                const SparseMatrix &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + matrix.numberOfNonZeros());

        for (SparseMatrix::const_iterator it(matrix.begin());
             it != matrix.end(); ++it) {
            m_triplets.push_back(Triplet(startingRow + it->row,
                                         startingColumn + it->column,
                                         it->value));
        }
    }

    void Triplets::addDiagonalMatrix(unsigned startingRow,
                                     unsigned startingColumn,
                                     double value,
                                     unsigned diagonalMatrixSize)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + diagonalMatrixSize);
        for (unsigned i = 0; i < diagonalMatrixSize; ++i) {
            m_triplets.push_back(Triplet(startingRow + i, startingColumn + i, value));
        }
    }

    unsigned Triplets::size() const { return m_triplets.size(); }

    std::vector<iDynTree::Triplet>::const_iterator Triplets::begin() const { return m_triplets.begin(); }
    std::vector<iDynTree::Triplet>::iterator Triplets::begin() { return m_triplets.begin(); }
    std::vector<iDynTree::Triplet>::const_iterator Triplets::end() const { return m_triplets.end(); }
    std::vector<iDynTree::Triplet>::iterator Triplets::end() { return m_triplets.end(); }

    void Triplets::pushTriplet(const iDynTree::Triplet &triplet)
    {
        m_triplets.push_back(triplet);
    }


    void Triplets::setSubMatrix(unsigned startingRow,
                                unsigned startingColumn,
                                const MatrixDynSize &matrix)
    {
        unsigned rows = matrix.rows();
        unsigned cols = matrix.cols();

        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (unsigned row = 0; row < rows; ++row) {
            for (unsigned col = 0; col < cols; ++col) {
                setTriplet(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void Triplets::setSubMatrix(unsigned startingRow,
                                unsigned startingColumn,
                                const SparseMatrix &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + matrix.numberOfNonZeros());

        for (SparseMatrix::const_iterator it(matrix.begin());
             it != matrix.end(); ++it) {
            setTriplet(Triplet(startingRow + it->row,
                               startingColumn + it->column,
                               it->value));
        }
    }

    void Triplets::setDiagonalMatrix(unsigned startingRow,
                                     unsigned startingColumn,
                                     double value,
                                     unsigned diagonalMatrixSize)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + diagonalMatrixSize);
        for (unsigned i = 0; i < diagonalMatrixSize; ++i) {
            setTriplet(Triplet(startingRow + i, startingColumn + i, value));
        }
    }


    void Triplets::setTriplet(const iDynTree::Triplet &triplet)
    {
        //search for triplet
        //TODO: vector is not the right choice as find can be very slow
        std::vector<Triplet>::iterator found = std::find(m_triplets.begin(), m_triplets.end(), triplet);
        if (found != m_triplets.end()) {
            *found = triplet;
        } else {
            m_triplets.push_back(triplet);
        }
    }



}
