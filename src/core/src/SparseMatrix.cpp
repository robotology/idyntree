/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "SparseMatrix.h"

#include "Triplets.h"

#include <cassert>
#include <sstream>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>
#include <iterator>

#ifndef UNUSED
#define UNUSED(x) (void)(sizeof((x), 0))
#endif

namespace iDynTree {

    // MARK: - generic SparseMatrix implementation

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::SparseMatrix() : SparseMatrix(0, 0) {}

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::SparseMatrix(std::size_t rows, std::size_t cols)
    : SparseMatrix(rows, cols, iDynTree::VectorDynSize())
    { }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::initializeMatrix(std::size_t outerSize, const double* vector, std::size_t vectorSize)
    {
        m_outerStarts.assign(outerSize + 1, 0);

        for (std::size_t i = 0; i < vectorSize; ++i) {
            m_allocatedSize += vector[i];
        }

        m_values.reserve(m_allocatedSize);

        // Inner indeces has same size of values
        m_innerIndices.reserve(m_allocatedSize);
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::~SparseMatrix() {}

    template <iDynTree::MatrixStorageOrdering ordering>
    template <iDynTree::MatrixStorageOrdering otherOrdering>
    SparseMatrix<ordering>::SparseMatrix(const SparseMatrix<otherOrdering>& other)
    : m_values(other.m_values)
    , m_innerIndices(other.m_innerIndices)
    , m_outerStarts(other.m_outerStarts)
    , m_allocatedSize(other.m_allocatedSize)
    , m_rows(other.m_rows)
    , m_columns(other.m_columns) {}

    template <iDynTree::MatrixStorageOrdering ordering>
    std::size_t SparseMatrix<ordering>::numberOfNonZeros() const
    {
        return m_values.size();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    template <iDynTree::MatrixStorageOrdering otherOrdering>
    SparseMatrix<ordering>& SparseMatrix<ordering>::operator=(const SparseMatrix<otherOrdering>& other)
    {
        if (this == &other) return *this;

        m_values = other.m_values;
        m_innerIndices = other.m_innerIndices;
        m_outerStarts = other.m_outerStarts;
        m_allocatedSize = other.m_allocatedSize;
        m_rows = other.m_rows;
        m_columns = other.m_columns;
        return *this;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::resize(std::size_t rows, std::size_t columns)
    {
        VectorDynSize empty; //this does not allocate memory
        resize(rows, columns, empty);
    }


    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::reserve(std::size_t nonZeroElements)
    {
        if (nonZeroElements <= m_allocatedSize) return; //do nothing

        m_values.reserve(nonZeroElements);
        m_innerIndices.reserve(nonZeroElements);
        m_allocatedSize = nonZeroElements;

    }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::zero()
    {
        //zero: simply clear
        m_values.resize(0);
        m_innerIndices.resize(0);
        m_outerStarts.assign(m_outerStarts.size(), 0);
    }

    template <MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::valueIndexForOuterAndInnerIndices(std::size_t outerIndex, std::size_t innerIndex, std::size_t& valueIndex) const
    {
        //We can use std::lower_bound to between rowNZIndex and rowNZIndex + rowNNZ
        //They are already sorted. The only critical point is if we have -1 in the matrix
        //Which right now it does not apply
        int outerBegin = m_outerStarts[outerIndex];
        int outerEnd = m_outerStarts[outerIndex + 1];
        std::vector<int>::const_iterator innerVectorBegin = m_innerIndices.begin();

        //initialize the return value to be the first element of the row
        valueIndex = outerBegin;
        if (outerEnd - outerBegin == 0) {
            //empty block, avoid searching
            return false;
        }

        std::vector<int>::const_iterator foundIndex = std::lower_bound(innerVectorBegin + outerBegin,
                                                                       innerVectorBegin + outerEnd, innerIndex);

        //Compute the index of the first element next or equal to the one we
        //were looking for

        valueIndex = std::distance(innerVectorBegin, foundIndex);
        if (foundIndex == innerVectorBegin + outerEnd) {
            //not found
            //return the index of the last element of the row
            //as we have to put the element as last element
            return false;
        }

        if (*foundIndex >= 0
            && static_cast<std::size_t>(*foundIndex) == innerIndex) {
            //found
            return true;
        }

        return false;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    std::size_t SparseMatrix<ordering>::insert(std::size_t outerIndex, std::size_t innerIndex, double value)
    {
        //first: check if there is space in the arrays
        if (m_allocatedSize <= m_values.size()) {
            reserve(m_values.size() + 10);
        }

        //find insertion position
        std::size_t insertionIndex = 0;
        if (valueIndexForOuterAndInnerIndices(outerIndex, innerIndex, insertionIndex)) {
            //???: what if the element exists alredy in the matrix?
            return insertionIndex;
        }

        //I found the index. Now I have to shift to the right the values and inner elements
        m_values.resize(m_values.size() + 1);
        m_innerIndices.resize(m_innerIndices.size() + 1);

        for (std::size_t i = m_values.size() - 1; i > insertionIndex && i > 0; --i) {
            m_values(i) = m_values(i - 1);
            m_innerIndices[i] = m_innerIndices[i - 1];
        }

        m_values(insertionIndex) = value;
        m_innerIndices[insertionIndex] = innerIndex;
        //update row NNZ
        for (std::size_t nextOuterIndex = outerIndex; nextOuterIndex < m_outerStarts.size() - 1; ++nextOuterIndex) {
            m_outerStarts[nextOuterIndex + 1]++;
        }

        return insertionIndex;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::setFromConstTriplets(const iDynTree::Triplets& triplets)
    {
        iDynTree::Triplets copy(triplets);
        setFromTriplets(copy);
    }


    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering> SparseMatrix<ordering>::sparseMatrixFromTriplets(std::size_t rows,
                                                                            std::size_t cols,
                                                                            const iDynTree::Triplets& nonZeroElements)
    {
        SparseMatrix newMatrix(rows, cols);
        newMatrix.setFromConstTriplets(nonZeroElements);
        return newMatrix;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    std::size_t SparseMatrix<ordering>::rows() const { return m_rows; }

    template <iDynTree::MatrixStorageOrdering ordering>
    std::size_t SparseMatrix<ordering>::columns() const { return m_columns; }

    template <iDynTree::MatrixStorageOrdering ordering>
    double * SparseMatrix<ordering>::valuesBuffer() { return m_values.data(); }

    template <iDynTree::MatrixStorageOrdering ordering>
    double const * SparseMatrix<ordering>::valuesBuffer() const { return m_values.data(); }

    template <iDynTree::MatrixStorageOrdering ordering>
    int * SparseMatrix<ordering>::innerIndicesBuffer()
    {
        return m_innerIndices.data();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    int const * SparseMatrix<ordering>::innerIndicesBuffer() const
    {
        return m_innerIndices.data();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    int * SparseMatrix<ordering>::outerIndicesBuffer()
    {
        return m_outerStarts.data();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    int const * SparseMatrix<ordering>::outerIndicesBuffer() const
    {
        return m_outerStarts.data();
    }


    template <iDynTree::MatrixStorageOrdering ordering>
    std::string SparseMatrix<ordering>::description(bool fullMatrix) const
    {
        std::ostringstream stream;
        if (!fullMatrix) {
            for (const_iterator it = begin(); it != end(); ++it) {
                stream << it->value << "(" << it->row << ", " << it->column << ") ";
            }
        } else {
            for (std::size_t row = 0; row < rows(); ++row) {
                for (std::size_t col = 0; col < columns(); ++col) {
                    stream << this->operator()(row, col) << " ";
                }
                stream << std::endl;
            }

        }
        return stream.str();
    }

#ifndef NDEBUG
    template <iDynTree::MatrixStorageOrdering ordering>
    std::string SparseMatrix<ordering>::internalDescription() const
    {
        std::ostringstream stream;
        stream << "Values: \n";
        for (std::size_t i = 0; i < m_values.size(); ++i) {
            stream << m_values(i) << " ";
        }
        stream << "\nInner indices: \n";
        for (std::size_t i = 0; i < m_values.size(); ++i) {
            stream << m_innerIndices[i] << " ";
        }
        stream << "\nOuter indices: \n";
        for (std::size_t i = 0; i < m_rows + 1; ++i) {
            stream << m_outerStarts[i] << " ";
        }
        stream << "\n";
        return stream.str();
    }
#endif

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::iterator SparseMatrix<ordering>::begin()
    {
        return iterator(*this);
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::const_iterator SparseMatrix<ordering>::begin() const
    {
        return const_iterator(*this);
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::iterator SparseMatrix<ordering>::end()
    {
        iterator it(*this, false);
        (&it)->m_index = -1;
        return it;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::const_iterator SparseMatrix<ordering>::end() const
    {
        const_iterator it(*this, false);
        (&it)->m_index = -1;
        return it;
    }

    // MARK: - Row-Major implementation
    template <>
    SparseMatrix<iDynTree::RowMajor>::SparseMatrix(std::size_t rows, std::size_t cols, const iDynTree::VectorDynSize& memoryReserveDescription)
    : m_allocatedSize(0)
    , m_rows(rows)
    , m_columns(cols)
    {
        initializeMatrix(rows, memoryReserveDescription.data(), memoryReserveDescription.size());
    }

    template <>
    double SparseMatrix<iDynTree::RowMajor>::operator()(std::size_t row, std::size_t col) const
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        std::size_t index = 0;
        double value = 0;
        if (valueIndexForOuterAndInnerIndices(row, col, index)) {
            value = m_values(index);
        }
        return value;
    }

    template <>
    double& SparseMatrix<iDynTree::RowMajor>::operator()(std::size_t row, std::size_t col)
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        std::size_t index = 0;
        if (valueIndexForOuterAndInnerIndices(row, col, index)) {
            return m_values(index);
        } else {
            return m_values(insert(row, col, 0));
        }
    }

    template <>
    void SparseMatrix<iDynTree::RowMajor>::resize(std::size_t rows, std::size_t columns, const iDynTree::VectorDynSize &columnNNZInformation)
    {
        //Avoid destroying the matrix if the size is the same
        if (m_rows == rows && m_columns == columns)
            return;

        m_rows = rows;
        m_columns = columns;

        initializeMatrix(rows, columnNNZInformation.data(), columnNNZInformation.size());
    }

    template <>
    void SparseMatrix<iDynTree::RowMajor>::setFromTriplets(iDynTree::Triplets& triplets)
    {
        if (triplets.size() == 0) return;

        //Get number of NZ and reserve buffers O(1) : size of compressed vector
        //We can overestimate with the size of triplets
        reserve(triplets.size());

        //Fastest way is to order by row and column N*log2(N)
        std::sort(triplets.begin(), triplets.end(), Triplet::rowMajorCompare);

        //now is a simple insert O(N) +
        //find to remove duplicates
        //Note: find is useless if array is sorted
        //Resize to maximum value. Will shrink at the end
        m_values.resize(triplets.size());
        m_innerIndices.resize(triplets.size());
        m_outerStarts.assign(m_rows + 1, 0); //reset vector


        std::size_t lastRow = 0;
        std::size_t lastColumn = 0;
        std::size_t innerIndex = 0;
        std::size_t lastIndex = innerIndex;
        m_values(0) = 0; //initialize the first element

        for (std::vector<Triplet>::const_iterator iterator(triplets.begin());
             iterator != triplets.end(); ++iterator) {
            // As triplets are ordered, only subsequent elements can be equal
            if (lastRow == iterator->row && lastColumn == iterator->column) {
                //Adjust for the first element
                //this should happen only the first time
                m_values(lastIndex) += iterator->value;
                //innerIndex should point to the next element
                //If the next element is at the same position, innerIndex will be ignored,
                //otherwise it will point to the next (free) element
                innerIndex = lastIndex + 1;
                continue;
            }


            //if current row is different from lastRow
            //I have to update the outerStarts vector
            if (lastRow != iterator->row) {
                for (std::vector<int>::iterator outerIt(m_outerStarts.begin() + lastRow + 1);
                     outerIt <= m_outerStarts.begin() + iterator->row; ++outerIt) {
                    *outerIt = innerIndex;
                }
                lastRow = iterator->row;
            }
            m_values(innerIndex) = iterator->value;
            m_innerIndices[innerIndex] = iterator->column;
            lastIndex = innerIndex;
            //increment index as this should always point to the next element
            ++innerIndex;
            lastColumn = iterator->column;

        }
        if (lastRow < m_rows) {
            for (std::vector<int>::iterator outerIt(m_outerStarts.begin() + lastRow + 1);
                 outerIt < m_outerStarts.end(); ++outerIt) {
                *outerIt = innerIndex;
            }

        }

        //Shrink containers
        m_values.resize(innerIndex);
        m_innerIndices.resize(innerIndex);

    }

    template <>
    template <>
    SparseMatrix<iDynTree::RowMajor>::SparseMatrix(const SparseMatrix<iDynTree::ColumnMajor>& other)
    : m_allocatedSize(0)
    , m_rows(other.rows())
    , m_columns(other.columns())
    {
        iDynTree::Triplets oldTriplets;
        oldTriplets.reserve(other.numberOfNonZeros());
        oldTriplets.addSubMatrix(0, 0, other);

        setFromTriplets(oldTriplets);
    }

    template <>
    template <>
    SparseMatrix<iDynTree::RowMajor>& SparseMatrix<iDynTree::RowMajor>::operator=(const SparseMatrix<iDynTree::ColumnMajor>& other)
    {
        resize(other.rows(), other.columns());
        iDynTree::Triplets oldTriplets;
        oldTriplets.reserve(other.numberOfNonZeros());
        oldTriplets.addSubMatrix(0, 0, other);

        setFromTriplets(oldTriplets);

        return *this;
    }

    // MARK: - Column-Major implementation
    template <>
    SparseMatrix<iDynTree::ColumnMajor>::SparseMatrix(std::size_t rows, std::size_t cols, const iDynTree::VectorDynSize& memoryReserveDescription)
    : m_allocatedSize(0)
    , m_rows(rows)
    , m_columns(cols)
    {
        initializeMatrix(cols, memoryReserveDescription.data(), memoryReserveDescription.size());
    }

    template <>
    double SparseMatrix<iDynTree::ColumnMajor>::operator()(std::size_t row, std::size_t col) const
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        std::size_t index = 0;
        double value = 0;
        if (valueIndexForOuterAndInnerIndices(col, row, index)) {
            value = m_values(index);
        }
        return value;
    }

    template <>
    double& SparseMatrix<iDynTree::ColumnMajor>::operator()(std::size_t row, std::size_t col)
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        std::size_t index = 0;
        if (valueIndexForOuterAndInnerIndices(col, row, index)) {
            return m_values(index);
        } else {
            return m_values(insert(col, row, 0));
        }
    }

    template <>
    void SparseMatrix<iDynTree::ColumnMajor>::resize(std::size_t rows, std::size_t columns, const iDynTree::VectorDynSize &columnNNZInformation)
    {
        //Avoid destroying the matrix if the size is the same
        if (m_rows == rows && m_columns == columns)
            return;

        m_rows = rows;
        m_columns = columns;

        initializeMatrix(columns, columnNNZInformation.data(), columnNNZInformation.size());
    }

    template <>
    void SparseMatrix<iDynTree::ColumnMajor>::setFromTriplets(iDynTree::Triplets& triplets)
    {
        if (triplets.size() == 0) return;

        //Get number of NZ and reserve buffers O(1) : size of compressed vector
        //We can overestimate with the size of triplets
        reserve(triplets.size());

        //Fastest way is to order by row and column N*log2(N)
        std::sort(triplets.begin(), triplets.end(), Triplet::columnMajorCompare);

        //now is a simple insert O(N) +
        //find to remove duplicates
        //Note: find is useless if array is sorted
        //Resize to maximum value. Will shrink at the end
        m_values.resize(triplets.size());
        m_innerIndices.resize(triplets.size());
        m_outerStarts.assign(m_columns + 1, 0); //reset vector


        std::size_t lastRow = 0;
        std::size_t lastColumn = 0;
        std::size_t innerIndex = 0;
        std::size_t lastIndex = innerIndex;
        m_values(0) = 0; //initialize the first element

        for (std::vector<Triplet>::const_iterator iterator(triplets.begin());
             iterator != triplets.end(); ++iterator) {
            // As triplets are ordered, only subsequent elements can be equal
            if (lastRow == iterator->row && lastColumn == iterator->column) {
                //Adjust for the first element
                //this should happen only the first time
                m_values(lastIndex) += iterator->value;
                //innerIndex should point to the next element
                //If the next element is at the same position, innerIndex will be ignored,
                //otherwise it will point to the next (free) element
                innerIndex = lastIndex + 1;
                continue;
            }


            //if current row is different from lastRow
            //I have to update the outerStarts vector
            if (lastColumn != iterator->column) {
                for (std::vector<int>::iterator outerIt(m_outerStarts.begin() + lastColumn + 1);
                     outerIt <= m_outerStarts.begin() + iterator->column; ++outerIt) {
                    *outerIt = innerIndex;
                }
                lastColumn = iterator->column;
            }
            m_values(innerIndex) = iterator->value;
            m_innerIndices[innerIndex] = iterator->row;
            lastIndex = innerIndex;
            //increment index as this should always point to the next element
            ++innerIndex;
            lastRow = iterator->row;

        }
        if (lastColumn < m_columns) {
            for (std::vector<int>::iterator outerIt(m_outerStarts.begin() + lastColumn + 1);
                 outerIt < m_outerStarts.end(); ++outerIt) {
                *outerIt = innerIndex;
            }

        }

        //Shrink containers
        m_values.resize(innerIndex);
        m_innerIndices.resize(innerIndex);

    }

    template <>
    template <>
    SparseMatrix<iDynTree::ColumnMajor>::SparseMatrix(const SparseMatrix<iDynTree::RowMajor>& other)
    : m_allocatedSize(0)
    , m_rows(other.rows())
    , m_columns(other.columns())
    {
        iDynTree::Triplets oldTriplets;
        oldTriplets.reserve(other.numberOfNonZeros());
        oldTriplets.addSubMatrix(0, 0, other);

        setFromTriplets(oldTriplets);
    }

    template <>
    template <>
    SparseMatrix<iDynTree::ColumnMajor>& SparseMatrix<iDynTree::ColumnMajor>::operator=(const SparseMatrix<iDynTree::RowMajor>& other)
    {
        resize(other.rows(), other.columns());
        iDynTree::Triplets oldTriplets;
        oldTriplets.reserve(other.numberOfNonZeros());
        oldTriplets.addSubMatrix(0, 0, other);

        setFromTriplets(oldTriplets);

        return *this;
    }

    // MARK: - Iterator implementation

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::Iterator::TripletRef::TripletRef(std::size_t row, std::size_t column, double *value)
    : m_row(row)
    , m_column(column)
    , m_value(value) {}

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::Iterator::Iterator(iDynTree::SparseMatrix<ordering> &matrix, bool valid)
    : m_matrix(matrix)
    , m_index(-1)
    , m_currentTriplet(-1, -1, 0)
    , m_nonZerosInOuterDirection(1) //to initialize the counter
    {
        if (matrix.m_values.size() == 0 || !valid) return;
        m_index = 0;

        updateTriplet();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::Iterator::updateTriplet()
    {
        assert(false);
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::Iterator& SparseMatrix<ordering>::Iterator::operator++()
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        m_index++;
        if (static_cast<std::size_t>(m_index) >= m_matrix.m_values.size()) {
            //Out of range
            m_index = -1;
        } else {
            updateTriplet();
        }

        return *this;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::Iterator SparseMatrix<ordering>::Iterator::operator++(int)
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        Iterator newIterator(*this);
        ++newIterator;
        return newIterator;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::Iterator::operator==(const Iterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::Iterator::operator==(const ConstIterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::Iterator::reference SparseMatrix<ordering>::Iterator::operator*()
    {
        return m_currentTriplet;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::Iterator::pointer SparseMatrix<ordering>::Iterator::operator->()
    {
        return &m_currentTriplet;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::Iterator::isValid() const
    {
        return m_index >= 0
        && true; //TODO: check if we are < than end or >= begin
    }

    template <>
    void SparseMatrix<iDynTree::RowMajor>::Iterator::updateTriplet()
    {
        m_currentTriplet.m_value = &(m_matrix.m_values(m_index));
        m_currentTriplet.m_column = m_matrix.m_innerIndices[m_index];

        if (--m_nonZerosInOuterDirection <= 0) {
            //increment row
            m_currentTriplet.m_row++;
            while (static_cast<std::size_t>(m_currentTriplet.m_row) < m_matrix.rows()) {
                //compute row NNZ
                m_nonZerosInOuterDirection = m_matrix.m_outerStarts[m_currentTriplet.m_row + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.m_row];
                if (m_nonZerosInOuterDirection > 0)
                    break;

                //increment row
                m_currentTriplet.m_row++;
            }
        }
    }

    template <>
    void SparseMatrix<iDynTree::ColumnMajor>::Iterator::updateTriplet()
    {
        m_currentTriplet.m_value = &(m_matrix.m_values(m_index));
        m_currentTriplet.m_row = m_matrix.m_innerIndices[m_index];

        if (--m_nonZerosInOuterDirection <= 0) {
            m_currentTriplet.m_column++;
            while (static_cast<std::size_t>(m_currentTriplet.m_column) < m_matrix.columns()) {
                //compute row NNZ
                m_nonZerosInOuterDirection = m_matrix.m_outerStarts[m_currentTriplet.m_column + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.m_column];
                if (m_nonZerosInOuterDirection > 0)
                    break;

                //increment row
                m_currentTriplet.m_column++;
            }
        }
    }


    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::ConstIterator::ConstIterator(const iDynTree::SparseMatrix<ordering> &matrix, bool valid)
    : m_matrix(matrix)
    , m_index(-1)
    , m_currentTriplet(-1, -1, 0)
    , m_nonZerosInOuterDirection(1) //to initialize the counter
    {
        if (matrix.m_values.size() == 0 || !valid) return;
        m_index = 0;

        updateTriplet();
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    SparseMatrix<ordering>::ConstIterator::ConstIterator(const typename SparseMatrix<ordering>::Iterator& iterator)
    : m_matrix(iterator.m_matrix)
    , m_index(iterator.m_index)
    , m_currentTriplet(iterator.m_currentTriplet.row(), iterator.m_currentTriplet.column(), iterator.isValid() ? iterator.m_currentTriplet.value() : 0)
    , m_nonZerosInOuterDirection(iterator.m_nonZerosInOuterDirection) { }

    template <iDynTree::MatrixStorageOrdering ordering>
    void SparseMatrix<ordering>::ConstIterator::updateTriplet()
    {
        assert(false);
    }

    template <>
    void SparseMatrix<iDynTree::RowMajor>::ConstIterator::updateTriplet()
    {
        m_currentTriplet.value = m_matrix.m_values(m_index);
        m_currentTriplet.column = m_matrix.m_innerIndices[m_index];

        if (--m_nonZerosInOuterDirection <= 0) {
            //increment row
            m_currentTriplet.row++;
            while (static_cast<std::size_t>(m_currentTriplet.row) < m_matrix.rows()) {
                //compute row NNZ
                m_nonZerosInOuterDirection = m_matrix.m_outerStarts[m_currentTriplet.row + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.row];
                if (m_nonZerosInOuterDirection > 0)
                    break;

                //increment row
                m_currentTriplet.row++;
            }
        }
    }

    template <>
    void SparseMatrix<iDynTree::ColumnMajor>::ConstIterator::updateTriplet()
    {
        m_currentTriplet.value = m_matrix.m_values(m_index);
        m_currentTriplet.row = m_matrix.m_innerIndices[m_index];

        if (--m_nonZerosInOuterDirection <= 0) {
            //increment row
            m_currentTriplet.column++;
            while (static_cast<std::size_t>(m_currentTriplet.column) < m_matrix.columns()) {
                //compute row NNZ
                m_nonZerosInOuterDirection = m_matrix.m_outerStarts[m_currentTriplet.column + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.column];
                if (m_nonZerosInOuterDirection > 0)
                    break;

                //increment row
                m_currentTriplet.column++;
            }
        }
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::ConstIterator& SparseMatrix<ordering>::ConstIterator::operator++()
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        m_index++;
        if (static_cast<std::size_t>(m_index) >= m_matrix.m_values.size()) {
            //Out of range
            m_index = -1;
        } else {
            updateTriplet();
        }

        return *this;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::ConstIterator SparseMatrix<ordering>::ConstIterator::operator++(int)
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        ConstIterator newIterator(*this);
        ++newIterator;
        return newIterator;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::ConstIterator::operator==(const ConstIterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    bool SparseMatrix<ordering>::ConstIterator::operator==(const Iterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::ConstIterator::reference SparseMatrix<ordering>::ConstIterator::operator*()
    {
        return m_currentTriplet;
    }

    template <iDynTree::MatrixStorageOrdering ordering>
    typename SparseMatrix<ordering>::ConstIterator::pointer SparseMatrix<ordering>::ConstIterator::operator->()
    {
        return &m_currentTriplet;
    }

    template <iDynTree::MatrixStorageOrdering ordering>

    bool SparseMatrix<ordering>::ConstIterator::isValid() const
    {
        return m_index >= 0
        && true; //TODO: check if we are < than end or >= begin
    }

}

// MARK: - Explicit instantiation of available templates
template class iDynTree::SparseMatrix<iDynTree::RowMajor>;
template class iDynTree::SparseMatrix<iDynTree::ColumnMajor>;
