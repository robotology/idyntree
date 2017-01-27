/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro, Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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

#if __cplusplus > 199711L

    SparseMatrix::SparseMatrix() : SparseMatrix(0, 0) {}

    SparseMatrix::SparseMatrix(unsigned rows, unsigned cols)
    : SparseMatrix(rows, cols, iDynTree::VectorDynSize())
    { }

    SparseMatrix::SparseMatrix(unsigned rows, unsigned cols, const iDynTree::VectorDynSize& memoryReserveDescription)
    : m_outerStarts(rows + 1, 0)
    , m_rows(rows)
    , m_columns(cols)
    , m_ordering(RowMajor)
    {
        initializeMatrix(rows, cols, memoryReserveDescription.data(), memoryReserveDescription.size());
    }
#else
    SparseMatrix::SparseMatrix()
    {
        initializeMatrix(0, 0, 0, 0);
    }

    SparseMatrix::SparseMatrix(unsigned rows, unsigned cols)
    {
        initializeMatrix(rows, cols, 0, 0);
    }

    SparseMatrix::SparseMatrix(unsigned rows, unsigned cols, const iDynTree::VectorDynSize& memoryReserveDescription)
    {
        initializeMatrix(rows, cols, memoryReserveDescription.data(), memoryReserveDescription.size());
    }

#endif

    void SparseMatrix::initializeMatrix(unsigned rows, unsigned cols, const double *vector, unsigned vectorSize)
    {
#if __cplusplus <= 199711L
        m_rows = rows;
        m_columns = cols;
        m_ordering = RowMajor;

        //Outer starts has same size of number of rows + 1
        m_outerStarts = new int[rows + 1];
        if (!m_outerStarts) {
            //TODO: throw exception?
        }
        memset(m_outerStarts, 0, sizeof(int) * (rows + 1));
#else
        UNUSED(rows);
        UNUSED(cols);
#endif

        
        m_allocatedSize = 0;
        for (unsigned i = 0; i < vectorSize; ++i) {
            m_allocatedSize += vector[i];
        }

        m_values.reserve(m_allocatedSize);

#if __cplusplus > 199711L
        // Inner indeces has same size of values
        m_innerIndeces.reserve(m_allocatedSize);
#else
        m_innerIndeces = new int[m_allocatedSize];
#endif

    }

    SparseMatrix::~SparseMatrix()
    {
#if __cplusplus <= 199711L
        delete [] m_innerIndeces;
        m_innerIndeces = 0;
        delete [] m_outerStarts;
        m_outerStarts = 0;
#endif
    }
#if __cplusplus <= 199711L
    SparseMatrix::SparseMatrix(const SparseMatrix &matrix)
    : m_values(matrix.m_values)
    , m_innerIndeces(0)
    , m_outerStarts(0)
    , m_allocatedSize(matrix.m_allocatedSize)
    , m_rows(matrix.m_rows)
    , m_columns(matrix.m_columns)
    , m_ordering(matrix.m_ordering)
    {
        m_innerIndeces = new int[m_allocatedSize];
        m_outerStarts = new int[m_rows + 1];
        std::memcpy(m_innerIndeces, matrix.m_innerIndeces, sizeof(int) * m_values.size());
        std::memcpy(m_outerStarts, matrix.m_outerStarts, sizeof(int) * (m_rows + 1));
        //reserve more memory for m_values if needed
        m_values.reserve(m_allocatedSize);

    }

    SparseMatrix& SparseMatrix::operator=(const SparseMatrix &matrix)
    {
        if (this == &matrix) return *this;
        //cleanup memory
        delete [] m_innerIndeces;
        delete [] m_outerStarts;
        //allocate new memory
        m_rows = matrix.m_rows;
        m_columns = matrix.m_columns;
        //copy outer start
        m_outerStarts = new int[m_rows + 1];
        std::memcpy(m_outerStarts, matrix.m_outerStarts, sizeof(int) * (m_rows + 1));
        //copy values
        m_allocatedSize = matrix.m_allocatedSize;
        m_values.reserve(m_allocatedSize);
        m_values = matrix.m_values;
        //copy inner indeces
        m_innerIndeces = new int[m_allocatedSize];
        std::memcpy(m_innerIndeces, matrix.m_innerIndeces, sizeof(int) * m_values.size());
        //copy ordering
        m_ordering = matrix.m_ordering;

        return *this;
    }

#endif

    bool SparseMatrix::valueIndex(unsigned row, unsigned col, unsigned &rowNZIndex) const
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());
        //TODO: more fast search?

        //find the row and how many zeros are in that row
        rowNZIndex = m_outerStarts[row];
        unsigned rowNNZ = m_outerStarts[row + 1] - rowNZIndex;

        //find inner position
        for (unsigned i = rowNZIndex; i < rowNZIndex + rowNNZ; ++i) {
            if (m_innerIndeces[i] < 0) continue;
            if (col == static_cast<unsigned>(m_innerIndeces[i])) {
                rowNZIndex = i;
                return true;
            }
            if (static_cast<unsigned>(m_innerIndeces[i]) > col) {
                rowNZIndex = i;
                return false;
            }
        }
        rowNZIndex += rowNNZ;
        return false;
    }

    unsigned SparseMatrix::insert(unsigned int row, unsigned int col, double value)
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        //first: check if there is space in the arrays
        if (m_allocatedSize <= m_values.size()) {
            reserve(m_values.size() + 10);
        }

        //find insertion position
        unsigned insertionIndex = 0;
        if (valueIndex(row, col, insertionIndex)) {
            //???: what if the element exists alredy in the matrix?
            return insertionIndex;
        }

        //I found the index. Now I have to shift to the right the values and inner elements
        m_values.resize(m_values.size() + 1);
#if __cplusplus > 199711L
        m_innerIndeces.resize(m_innerIndeces.size() + 1);
#endif
        for (unsigned i = insertionIndex; i < m_values.size() - 1; ++i) {
            m_values(i + 1) = m_values(i);
            m_innerIndeces[i + 1] = m_innerIndeces[i];
        }
        m_values(insertionIndex) = value;
        m_innerIndeces[insertionIndex] = col;
        //update row NNZ
        for (unsigned rowIndex = row; rowIndex < rows(); ++rowIndex) {
            m_outerStarts[rowIndex + 1]++;
        }

        return insertionIndex;
    }


    
    unsigned SparseMatrix::numberOfNonZeros() const
    {
        return m_values.size();
    }

    unsigned SparseMatrix::nonZeroElementsForRowAtIndex(unsigned rowIndex) const
    {
        assert(rowIndex >= 0 && rowIndex < rows());
        return m_outerStarts[rowIndex + 1] - m_outerStarts[rowIndex];
    }

    void SparseMatrix::resize(unsigned rows, unsigned columns)
    {
        VectorDynSize empty; //this does not allocate memory
        resize(rows, columns, empty);
    }

    void SparseMatrix::resize(unsigned rows, unsigned columns, const iDynTree::VectorDynSize &columnNNZInformation)
    {
        //suppress unused parameter for now
        ((void)columnNNZInformation);
        m_rows = rows;
        m_columns = columns;

#if __cplusplus > 199711L
        m_outerStarts.resize(rows);
        m_outerStarts.assign(rows, 0);
#else
        delete [] m_outerStarts;
        m_outerStarts = new int[rows + 1];
        memset(m_outerStarts, 0, sizeof(int) * (rows + 1));
#endif
        //TODO: use the information on the NNZ to reserve memory
    }

    void SparseMatrix::reserve(unsigned nonZeroElements)
    {
        if (nonZeroElements <= m_allocatedSize) return; //do nothing

        m_values.reserve(nonZeroElements);
#if __cplusplus > 199711L
        m_innerIndeces.reserve(nonZeroElements);
#else
        int *newBuffer = new int[nonZeroElements];
        if (m_innerIndeces) {
            memcpy(newBuffer, m_innerIndeces, sizeof(int) * m_allocatedSize);
        }
        delete [] m_innerIndeces;
        m_innerIndeces = newBuffer;
#endif
        m_allocatedSize = nonZeroElements;

    }

    void SparseMatrix::zero()
    {
        //zero: simply clear
        m_values.resize(0);
#if __cplusplus > 199711L
        m_innerIndeces.resize(0);
        m_outerStarts.resize(0);
#else
#endif
    }


    void SparseMatrix::setFromConstTriplets(const iDynTree::Triplets& triplets)
    {
        iDynTree::Triplets copy(triplets);
        setFromTriplets(copy);
    }

    void SparseMatrix::setFromTriplets(iDynTree::Triplets& triplets)
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
#if __cplusplus > 199711L
        m_innerIndeces.resize(triplets.size());
        m_outerStarts.assign(m_rows + 1, 0); //reset vector
#else
        memset(m_outerStarts, 0, sizeof(int) * (m_rows + 1));
#endif

        //initializing the first element
//        std::iterator<std::forward_iterator_tag, Triplet> iterator(triplets.begin());
        std::vector<Triplet>::const_iterator iterator(triplets.begin());
        //saving coordinates of last inserted value
        unsigned lastRow = iterator->row, lastCol = iterator->column;
        //saving value
        m_values(0) = iterator->value;
        //and column
        m_innerIndeces[0] = iterator->column;

        //setting the NNZ of next row (only if we are not inserting in last row)
        //if (iterator->row + 1 < m_rows) m_outerStarts[iterator->row + 1] = 1;
        m_outerStarts[iterator->row + 1] = 1;

        //starting loop
        ++iterator;
        //saving last inserted index
        unsigned valueIndex = 0;
        for (; iterator != triplets.end(); ++iterator) {
            if (lastRow != iterator->row) {
                //initialize NNZ for row
                for (unsigned rowIndex = lastRow + 1;
                     rowIndex <= iterator->row && rowIndex < rows(); ++rowIndex) {
                    m_outerStarts[rowIndex + 1] = m_outerStarts[rowIndex];
                }
            }

            if (lastRow == iterator->row
                && lastCol == iterator->column) {
                //duplicate
                m_values(valueIndex) += iterator->value;
                continue;
            }

            //updating NNZ
            ++m_outerStarts[iterator->row + 1];

            //Different coordinate. Insert new value
            valueIndex++;
            m_values(valueIndex) = iterator->value;
            m_innerIndeces[valueIndex] = iterator->column;

            lastRow = iterator->row;
            lastCol = iterator->column;
        }

        //Shrink containers
        m_values.resize(valueIndex + 1);
#if __cplusplus > 199711L
        m_innerIndeces.resize(valueIndex + 1);
#endif
    }

    unsigned SparseMatrix::rows() const { return m_rows; }
    unsigned SparseMatrix::columns() const { return m_columns; }

    double SparseMatrix::operator()(unsigned int row, unsigned int col) const
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        unsigned index = 0;
        double value = 0;
        if (valueIndex(row, col, index)) {
            value = m_values(index);
        }
        return value;
    }

    double& SparseMatrix::operator()(unsigned int row, unsigned int col)
    {
        assert(row >= 0 && row < rows()
               && col >= 0 && col < columns());

        unsigned index = 0;
        if (valueIndex(row, col, index)) {
            return m_values(index);
        } else {
            return m_values(insert(row, col, 0));
        }
    }

    double * SparseMatrix::valuesBuffer() { return m_values.data(); }

    double const * SparseMatrix::valuesBuffer() const { return m_values.data(); }

    int * SparseMatrix::innerIndecesBuffer()
    {
#if __cplusplus > 199711L
        return m_innerIndeces.data();
#else
        return m_innerIndeces;
#endif
    }

    int const * SparseMatrix::innerIndecesBuffer() const
    {
#if __cplusplus > 199711L
        return m_innerIndeces.data();
#else
        return m_innerIndeces;
#endif
    }

    int * SparseMatrix::outerIndecesBuffer()
    {
#if __cplusplus > 199711L
        return m_outerStarts.data();
#else
        return m_outerStarts;
#endif
    }

    int const * SparseMatrix::outerIndecesBuffer() const
    {
#if __cplusplus > 199711L
        return m_outerStarts.data();
#else
        return m_outerStarts;
#endif
    }

    std::string SparseMatrix::description(bool fullMatrix) const
    {
        std::ostringstream stream;
        if (!fullMatrix) {
            for (const_iterator it = begin(); it != end(); ++it) {
                stream << it->value << "(" << it->row << ", " << it->column << ") ";
            }
        } else {
            for (unsigned row = 0; row < rows(); ++row) {
                for (unsigned col = 0; col < columns(); ++col) {
                    unsigned index = 0;
                    if (!valueIndex(row, col, index)) {
                        stream << "0 ";
                    } else {
                        stream << m_values(index) << " ";
                    }
                }
                stream << std::endl;
            }

        }
        return stream.str();
    }

#ifndef NDEBUG
    std::string SparseMatrix::internalDescription() const
    {
        std::ostringstream stream;
        stream << "Values: \n";
        for (unsigned i = 0; i < m_values.size(); ++i) {
            stream << m_values(i) << " ";
        }
        stream << "\nInner indeces: \n";
        for (unsigned i = 0; i < m_values.size(); ++i) {
            stream << m_innerIndeces[i] << " ";
        }
        stream << "\nOuter indeces: \n";
        for (unsigned i = 0; i < m_rows + 1; ++i) {
            stream << m_outerStarts[i] << " ";
        }
        stream << "\n";
        return stream.str();
    }
#endif

    SparseMatrix::iterator SparseMatrix::begin()
    {
        return iterator(*this);
    }

    SparseMatrix::const_iterator SparseMatrix::begin() const
    {
        return const_iterator(*this);
    }

    SparseMatrix::iterator SparseMatrix::end()
    {
        iterator it(*this, false);
        (&it)->m_index = -1;
        return it;
    }

    SparseMatrix::const_iterator SparseMatrix::end() const
    {
        const_iterator it(*this, false);
        (&it)->m_index = -1;
        return it;
    }

    void SparseMatrix::convertToColumnMajor(double *values, int *innerBuffer, int *outerBuffer) const
    {
        //empty matrix
        if (m_values.size() == 0) return;

        //assume buffers are allocated correctly
        //reset outer buffer
        std::memset(outerBuffer, 0, sizeof(int) * (m_columns + 1));

        //Insert each element by looking for the correct position
        //sorted insertion => O(N logN)
        //This is equivalent to a sort :)
        //The only think is that we sort a temporary array as we have to keep track
        //of the values assosiated with the column
        std::vector<Triplet> colMajorInnerContainer;
        colMajorInnerContainer.reserve(m_values.size());

        for (const_iterator it(begin()); it != end(); ++it) {
            colMajorInnerContainer.push_back(Triplet(it->row, it->column, it->value));
        }

        std::sort(colMajorInnerContainer.begin(), colMajorInnerContainer.end(), Triplet::columnMajorCompare);

        //Now we iterate on the vector and fill the outputs
        //We can directly fill the values and the row index
        std::vector<Triplet>::iterator it(colMajorInnerContainer.begin());
        std::vector<Triplet>::iterator begin = it;
        unsigned currentColumn = begin->column;
        unsigned currentNNZ = 0;
        for (; it != colMajorInnerContainer.end(); ++it) {
            unsigned index = std::distance(begin, it);
            currentNNZ++;

            if (currentColumn != it->column) {
                //this column is different from the previous one
                //Copy the current NNZ to all the columns till the new one
                //And start counting again the NNZ
                for (unsigned col = currentColumn + 1; col <= it->column; ++col) {
                    outerBuffer[col] = currentNNZ-1;
                }
//                outerBuffer[it->column] = currentNNZ - 1; //this is to compensate with the
                //current increment which is not of this column
                //and by the way this should be equivalent to
                //outerBuffer[it->column] = index
                currentColumn = it->column;
            }

            values[index] = it->value;
            innerBuffer[index] = it->row;
        }
        //update last value
        outerBuffer[columns()] = currentNNZ;
    }

    void SparseMatrix::convertFromColumnMajor(unsigned rows,
                                              unsigned columns,
                                              unsigned numberOfNonZeros,
                                              double * const values,
                                              int * const innerBuffer,
                                              int * const outerBuffer)
    {
        //reset size
        resize(rows, columns);

        //iterate over all values and fill a triplet list
        Triplets triplets;
        triplets.reserve(numberOfNonZeros);

        for (unsigned col = 0; col < columns; ++col) {
            //iterate over the rows nz
            for (int innerIndex = outerBuffer[col]; innerIndex < outerBuffer[col + 1]; ++innerIndex) {
                triplets.pushTriplet(Triplet(innerBuffer[innerIndex], col, values[innerIndex]));
            }
        }

        setFromTriplets(triplets);

    }

    // Iterator implementation

    SparseMatrix::Iterator::TripletRef::TripletRef(unsigned row, unsigned column, double *value)
    : m_row(row)
    , m_column(column)
    , m_value(value) {}

    SparseMatrix::Iterator::Iterator(iDynTree::SparseMatrix &matrix, bool valid)
    : m_matrix(matrix)
    , m_index(-1)
    , m_currentTriplet(-1, -1, 0)
    , m_nonZerosInRow(1) //to initialize the counter
    {
        if (matrix.m_values.size() == 0 || !valid) return;
        m_index = 0;

        updateTriplet();
    }

    void SparseMatrix::Iterator::updateTriplet()
    {
        m_currentTriplet.m_value = &(m_matrix.m_values(m_index));
        m_currentTriplet.m_column = m_matrix.m_innerIndeces[m_index];

        if (--m_nonZerosInRow <= 0) {
            //increment row
            m_currentTriplet.m_row++;
            while (static_cast<unsigned>(m_currentTriplet.m_row) < m_matrix.rows()) {
                //compute row NNZ
                m_nonZerosInRow = m_matrix.m_outerStarts[m_currentTriplet.m_row + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.m_row];
                if (m_nonZerosInRow > 0)
                    break;

                //increment row
                m_currentTriplet.m_row++;
            }
        }

    }

    SparseMatrix::Iterator& SparseMatrix::Iterator::operator++()
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        m_index++;
        if (static_cast<unsigned>(m_index) >= m_matrix.m_values.size()) {
            //Out of range
            m_index = -1;
        } else {
            updateTriplet();
        }

        return *this;
    }

    SparseMatrix::Iterator SparseMatrix::Iterator::operator++(int)
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        Iterator newIterator(*this);
        ++newIterator;
        return newIterator;
    }

//    SparseMatrix::Iterator& SparseMatrix::Iterator::operator--()
//    {
//        if (m_index < 0) {
//            //Iterator is not valid. We do nothing
//            return *this;
//        }
//        m_index--;
//        updateTriplet();
//
//        return *this;
//    }
//
//    SparseMatrix::Iterator SparseMatrix::Iterator::operator--(int)
//    {
//        if (m_index < 0) {
//            //Iterator is not valid. We do nothing
//            return *this;
//        }
//        Iterator newIterator(*this);
//        --newIterator;
//        return newIterator;
//    }

    bool SparseMatrix::Iterator::operator==(const Iterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    bool SparseMatrix::Iterator::operator==(const ConstIterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    SparseMatrix::Iterator::reference SparseMatrix::Iterator::operator*()
    {
        return m_currentTriplet;
    }

    SparseMatrix::Iterator::pointer SparseMatrix::Iterator::operator->()
    {
        return &m_currentTriplet;
    }
    
    bool SparseMatrix::Iterator::isValid() const
    {
        return m_index >= 0
        && true; //TODO: check if we are < than end or >= begin
    }

    SparseMatrix::ConstIterator::ConstIterator(const iDynTree::SparseMatrix &matrix, bool valid)
    : m_matrix(matrix)
    , m_index(-1)
    , m_currentTriplet(-1, -1, 0)
    , m_nonZerosInRow(1) //to initialize the counter
    {
        if (matrix.m_values.size() == 0 || !valid) return;
        m_index = 0;

        updateTriplet();
    }

    SparseMatrix::ConstIterator::ConstIterator(const SparseMatrix::Iterator& iterator)
    : m_matrix(iterator.m_matrix)
    , m_index(iterator.m_index)
    , m_currentTriplet(iterator.m_currentTriplet.row(), iterator.m_currentTriplet.column(), iterator.isValid() ? iterator.m_currentTriplet.value() : 0)
    , m_nonZerosInRow(iterator.m_nonZerosInRow) { }

    void SparseMatrix::ConstIterator::updateTriplet()
    {
        m_currentTriplet.value = m_matrix.m_values(m_index);
        m_currentTriplet.column = m_matrix.m_innerIndeces[m_index];

        if (--m_nonZerosInRow <= 0) {
            //increment row
            m_currentTriplet.row++;
            while (static_cast<unsigned>(m_currentTriplet.row) < m_matrix.rows()) {
                //compute row NNZ
                m_nonZerosInRow = m_matrix.m_outerStarts[m_currentTriplet.row + 1]
                - m_matrix.m_outerStarts[m_currentTriplet.row];
                if (m_nonZerosInRow > 0)
                    break;

                //increment row
                m_currentTriplet.row++;
            }
        }
    }

    SparseMatrix::ConstIterator& SparseMatrix::ConstIterator::operator++()
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        m_index++;
        if (static_cast<unsigned>(m_index) >= m_matrix.m_values.size()) {
            //Out of range
            m_index = -1;
        } else {
            updateTriplet();
        }

        return *this;
    }

    SparseMatrix::ConstIterator SparseMatrix::ConstIterator::operator++(int)
    {
        if (m_index < 0) {
            //Iterator is not valid. We do nothing
            return *this;
        }
        ConstIterator newIterator(*this);
        ++newIterator;
        return newIterator;
    }

//    SparseMatrix::ConstIterator& SparseMatrix::ConstIterator::operator--()
//    {
//        if (m_index < 0) {
//            //Iterator is not valid. We do nothing
//            return *this;
//        }
//        m_index--;
//        return *this;
//    }
//
//    SparseMatrix::ConstIterator SparseMatrix::ConstIterator::operator--(int)
//    {
//        if (m_index < 0) {
//            //Iterator is not valid. We do nothing
//            return *this;
//        }
//        ConstIterator newIterator(*this);
//        --newIterator;
//        return newIterator;
//    }

    bool SparseMatrix::ConstIterator::operator==(const ConstIterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    bool SparseMatrix::ConstIterator::operator==(const Iterator &it) const
    {
        return &m_matrix == &((&it)->m_matrix) //check that we are pointing to the same matrix
        && m_index == it.m_index;
    }

    SparseMatrix::ConstIterator::reference SparseMatrix::ConstIterator::operator*()
    {
        return m_currentTriplet;
    }

    SparseMatrix::ConstIterator::pointer SparseMatrix::ConstIterator::operator->()
    {
        return &m_currentTriplet;
    }

    bool SparseMatrix::ConstIterator::isValid() const
    {
        return m_index >= 0
        && true; //TODO: check if we are < than end or >= begin
    }

}
