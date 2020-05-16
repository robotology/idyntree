/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TRIPLETS_H
#define IDYNTREE_TRIPLETS_H

#include <iDynTree/Core/Utils.h>

#include <iterator>
#include <string>
#include <vector>


namespace iDynTree {
    class Triplet;
    class Triplets;

    class MatrixDynSize;
    template <unsigned rows, unsigned cols>
    class MatrixFixSize;

    template <MatrixStorageOrdering ordering>
    class SparseMatrix;

    struct IndexRange;
}

class iDynTree::Triplet
{
public:

    static bool rowMajorCompare(const iDynTree::Triplet& a, const iDynTree::Triplet &b);
    static bool columnMajorCompare(const iDynTree::Triplet& a, const iDynTree::Triplet &b);

    Triplet(unsigned row, unsigned column, double value);

    bool operator<(const iDynTree::Triplet&) const;
    bool operator==(const iDynTree::Triplet&) const;

    unsigned row;
    unsigned column;
    double value;
};

class iDynTree::Triplets
{
    //Understand if vector is the best solution
    std::vector<iDynTree::Triplet> m_triplets;

public:

    void reserve(unsigned size);
    void clear();

    template <unsigned rows, unsigned cols>
    inline void addSubMatrix(unsigned startingRow,
                             unsigned startingColumn,
                             const MatrixFixSize<rows, cols> &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (unsigned row = 0; row < rows; ++row) {
            for (unsigned col = 0; col < cols; ++col) {
                m_triplets.push_back(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void addSubMatrix(unsigned startingRow,
                      unsigned startingColumn,
                      const MatrixDynSize&);

    template <MatrixStorageOrdering ordering>
    void addSubMatrix(unsigned startingRow,
                      unsigned startingColumn,
                      const SparseMatrix<ordering>& matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + matrix.numberOfNonZeros());

        for (typename SparseMatrix<ordering>::const_iterator it(matrix.begin());
             it != matrix.end(); ++it) {
            m_triplets.push_back(Triplet(startingRow + it->row,
                                         startingColumn + it->column,
                                         it->value));
        }
    }

    inline void addDiagonalMatrix(IndexRange startingRow,
                                  IndexRange startingColumn,
                                  double value)
    {
        addDiagonalMatrix(startingRow.offset,
                          startingColumn.offset,
                          value,
                          startingRow.size);
    }

    void addDiagonalMatrix(unsigned startingRow,
                           unsigned startingColumn,
                           double value,
                           unsigned diagonalMatrixSize);

    void pushTriplet(const Triplet& triplet);

    template <unsigned rows, unsigned cols>
    inline void setSubMatrix(unsigned startingRow,
                             unsigned startingColumn,
                             const MatrixFixSize<rows, cols> &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (unsigned row = 0; row < rows; ++row) {
            for (unsigned col = 0; col < cols; ++col) {
                setTriplet(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void setSubMatrix(unsigned startingRow,
                      unsigned startingColumn,
                      const MatrixDynSize&);

    template <MatrixStorageOrdering ordering>
    void setSubMatrix(unsigned startingRow,
                      unsigned startingColumn,
                      const SparseMatrix<ordering>& matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + matrix.numberOfNonZeros());

        for (typename SparseMatrix<ordering>::const_iterator it(matrix.begin());
             it != matrix.end(); ++it) {
            setTriplet(Triplet(startingRow + it->row,
                               startingColumn + it->column,
                               it->value));
        }
    }

    inline void setDiagonalMatrix(IndexRange startingRow,
                                  IndexRange startingColumn,
                                  double value)
    {
        setDiagonalMatrix(startingRow.offset,
                          startingColumn.offset,
                          value,
                          startingRow.size);
    }

    void setDiagonalMatrix(unsigned startingRow,
                           unsigned startingColumn,
                           double value,
                           unsigned diagonalMatrixSize);


    void setTriplet(const Triplet& triplet);

    bool isEmpty() const;
    unsigned size() const;

    std::string description() const;

    typedef std::vector<iDynTree::Triplet>::iterator iterator;
    typedef std::vector<iDynTree::Triplet>::const_iterator const_iterator;

    //for now simply return the vector iterator and do not implement our own iterator
    const_iterator begin() const;
    iterator begin();
    const_iterator end() const;
    iterator end();

};


#endif /* end of include guard: IDYNTREE_TRIPLETS_H */
