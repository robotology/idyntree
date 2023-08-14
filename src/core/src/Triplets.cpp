// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "Triplets.h"
#include "MatrixDynSize.h"
#include "MatrixFixSize.h"
#include "SparseMatrix.h"

#include <algorithm>
#include <sstream>

namespace iDynTree {

    Triplet::Triplet(std::size_t _row, std::size_t _column, double _value)
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

    void Triplets::reserve(std::size_t size)
    {
        m_triplets.reserve(size);
    }

    void Triplets::clear()
    {
        m_triplets.clear();
    }

    void Triplets::addSubMatrix(std::size_t startingRow,
                                std::size_t startingColumn,
                                const MatrixDynSize &matrix)
    {
        std::size_t rows = matrix.rows();
        std::size_t cols = matrix.cols();

        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
                m_triplets.push_back(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void Triplets::addDiagonalMatrix(std::size_t startingRow,
                                     std::size_t startingColumn,
                                     double value,
                                     std::size_t diagonalMatrixSize)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + diagonalMatrixSize);
        for (std::size_t i = 0; i < diagonalMatrixSize; ++i) {
            m_triplets.push_back(Triplet(startingRow + i, startingColumn + i, value));
        }
    }

    bool Triplets::isEmpty() const { return m_triplets.empty(); }
    std::size_t Triplets::size() const { return m_triplets.size(); }

    std::vector<iDynTree::Triplet>::const_iterator Triplets::begin() const { return m_triplets.begin(); }
    std::vector<iDynTree::Triplet>::iterator Triplets::begin() { return m_triplets.begin(); }
    std::vector<iDynTree::Triplet>::const_iterator Triplets::end() const { return m_triplets.end(); }
    std::vector<iDynTree::Triplet>::iterator Triplets::end() { return m_triplets.end(); }

    void Triplets::pushTriplet(const iDynTree::Triplet &triplet)
    {
        m_triplets.push_back(triplet);
    }


    void Triplets::setSubMatrix(std::size_t startingRow,
                                std::size_t startingColumn,
                                const MatrixDynSize &matrix)
    {
        std::size_t rows = matrix.rows();
        std::size_t cols = matrix.cols();

        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
                setTriplet(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void Triplets::setDiagonalMatrix(std::size_t startingRow,
                                     std::size_t startingColumn,
                                     double value,
                                     std::size_t diagonalMatrixSize)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + diagonalMatrixSize);
        for (std::size_t i = 0; i < diagonalMatrixSize; ++i) {
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


    std::string Triplets::description() const
    {
        std::ostringstream str;
        for (const_iterator triplet(begin()); triplet != end(); ++triplet) {
            str << "(" << triplet->row << "," << triplet->column << ")" << triplet->value << std::endl;
        }
        return str.str();
    }



}
