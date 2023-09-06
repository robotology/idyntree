// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_TRIPLETS_H
#define IDYNTREE_TRIPLETS_H

#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/Utils.h>

#include <iterator>
#include <string>
#include <vector>


namespace iDynTree {
    class Triplet;
    class Triplets;

    class MatrixDynSize;

    template <MatrixStorageOrdering ordering>
    class SparseMatrix;

    struct IndexRange;
}

class iDynTree::Triplet
{
public:

    static bool rowMajorCompare(const iDynTree::Triplet& a, const iDynTree::Triplet &b);
    static bool columnMajorCompare(const iDynTree::Triplet& a, const iDynTree::Triplet &b);

    Triplet(std::size_t row, std::size_t column, double value);

    bool operator<(const iDynTree::Triplet&) const;
    bool operator==(const iDynTree::Triplet&) const;

    std::size_t row;
    std::size_t column;
    double value;
};

class iDynTree::Triplets
{
    //Understand if vector is the best solution
    std::vector<iDynTree::Triplet> m_triplets;

public:

    void reserve(std::size_t size);
    void clear();

    template <unsigned int rows, unsigned int cols>
    inline void addSubMatrix(std::size_t startingRow,
                             std::size_t startingColumn,
                             const MatrixFixSize<rows, cols> &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
                m_triplets.push_back(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void addSubMatrix(std::size_t startingRow,
                      std::size_t startingColumn,
                      const MatrixDynSize&);

    template <MatrixStorageOrdering ordering>
    void addSubMatrix(std::size_t startingRow,
                      std::size_t startingColumn,
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

    void addDiagonalMatrix(std::size_t startingRow,
                           std::size_t startingColumn,
                           double value,
                           std::size_t diagonalMatrixSize);

    void pushTriplet(const Triplet& triplet);

    template <std::size_t rows, std::size_t cols>
    inline void setSubMatrix(std::size_t startingRow,
                             std::size_t startingColumn,
                             const MatrixFixSize<rows, cols> &matrix)
    {
        //if enough memory has been reserved this should be a noop
        m_triplets.reserve(m_triplets.size() + (rows * cols));
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t col = 0; col < cols; ++col) {
                setTriplet(Triplet(startingRow + row, startingColumn + col, matrix(row, col)));
            }
        }
    }

    void setSubMatrix(std::size_t startingRow,
                      std::size_t startingColumn,
                      const MatrixDynSize&);

    template <MatrixStorageOrdering ordering>
    void setSubMatrix(std::size_t startingRow,
                      std::size_t startingColumn,
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

    void setDiagonalMatrix(std::size_t startingRow,
                           std::size_t startingColumn,
                           double value,
                           std::size_t diagonalMatrixSize);


    void setTriplet(const Triplet& triplet);

    bool isEmpty() const;
    std::size_t size() const;

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
