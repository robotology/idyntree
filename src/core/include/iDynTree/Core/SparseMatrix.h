/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro, Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPARSE_MATRIX_H
#define IDYNTREE_SPARSE_MATRIX_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/Utils.h>
#include <vector>

namespace iDynTree {
    class SparseMatrix;
    class Triplet;
    class Triplets;
}

/**
 * \brief Sparse Matrix class
 *
 * This class uses the Compressed Row Storage scheme
 * (see https://en.wikipedia.org/wiki/Sparse_matrix#Compressed_sparse_row_.28CSR.2C_CRS_or_Yale_format.29)
 * which is compatible with the format used in the Eigen library (by using Map).
 */
class iDynTree::SparseMatrix
{

public:
    enum StorageOrdering {
        RowMajor,
        ColumnMajor
    };

private:

    iDynTree::VectorDynSize m_values; /**< Contains all the non zero (NZ) elements */

    std::vector<int> m_innerIndices; /**< column index of the NZ elements  */
    std::vector<int> m_outerStarts; /**< for each row contains the index of the first NZ in the
                                     *   previous two arrays
                                     */


    unsigned m_allocatedSize; /**< size of the memory allocated for m_values and m_innerIndices */

    unsigned m_rows;
    unsigned m_columns;

    StorageOrdering m_ordering;

    void initializeMatrix(unsigned rows, unsigned cols, const double*, unsigned vectorSize);

    /**
     * Insert a new element at the specified position
     *
     * It returns the index in m_values of the inserted element
     * @param row row of the new element
     * @param col column of the new element
     * @param value value to be inserted
     * @return index in m_values of the inserted element
     */
    unsigned insert(unsigned row, unsigned col, double value);

    /**
     * Check if the element at row-col is present in the matrix.
     *
     * If it is present the function returns true and the index in the
     * out parameter index, otherwise it returns false and index will
     * contain the index of the next element in the value array
     *
     * @param[in] row row index
     * @param[in] col column index
     * @param[out] index if found contains the index, if not the index of the next element
     * @return true if the value has been found
     */
    bool valueIndex(unsigned row, unsigned col, unsigned &index) const;

public:

    /**
     * Creates an empty sparse matrix.
     */
    SparseMatrix();

    /**
     * Creates a zero sparse matrix with the specified dimensions
     *
     */
    SparseMatrix(unsigned rows, unsigned cols);


    SparseMatrix(unsigned rows, unsigned cols, const iDynTree::VectorDynSize& memoryReserveDescription);

    /**
     * Default destructor
     */
    ~SparseMatrix();

    /**
     * Returns the number of nonzero elements in this sparse matrix
     *
     * @return the number of non zero elements
     */
    unsigned numberOfNonZeros() const;


    unsigned nonZeroElementsForRowAtIndex(unsigned rowIndex) const;


    /**
     * Resize the matrix to the specified new dimensions
     *
     * \note the content of the matrix is not preserved in any case
     * \warning this function may perform memory allocation
     * @param rows the new number of rows of this matrix
     * @param columns the new number of columns of this matrix
     */
    void resize(unsigned rows, unsigned columns);

    /**
     * Resize the matrix to the specified new dimensions
     *
     * \note the content of the matrix is not preserved in any case
     * \warning this function may perform memory allocation
     * @param rows the new number of rows of this matrix
     * @param columns the new number of columns of this matrix
     * @param columnNNZInformation information on the NNZ for each column, used to reserve memory in advance
     */
    void resize(unsigned rows, unsigned columns, const iDynTree::VectorDynSize &columnNNZInformation);

    void reserve(unsigned nonZeroElements);


    /**
     * Set the sparse matrix to be zero
     *
     * @note In C++11 it is not guaranteed that this function performs no memory allocation,
     * depending on the standard library implementation.
     */
    void zero();


    /**
     * Sets the content of this sparse matrix to the content of triplets
     *
     * This function does not set the dimensions of the matrix
     * which must be set beforehand. If the dimensions are wrong the
     * behaviour is undefined.
     *
     * \note duplicate elements in triplets will be summed up
     * \note the content of this matrix will be totally overwritten
     *
     * \warning this function performs a copy of the triplets parameter so
     * that it does not modify the input parameter.
     * Use setFromTriplets(iDynTree::Triplets& triplets) if you accept to have
     * triplets modified.
     *
     * @param triplets triplets containing the non zero elements
     */
    void setFromConstTriplets(const iDynTree::Triplets& triplets);

    /**
     * Sets the content of this sparse matrix to the content of triplets
     *
     * This function does not set the dimensions of the matrix
     * which must be set beforehand. If the dimensions are wrong the
     * behaviour is undefined.
     *
     * \note duplicate elements in triplets will be summed up
     * \note the content of this matrix will be totally overwritten
     *
     * \warning this function modifies the input parameter triplets.
     * Use setFromConstTriplets(const iDynTree::Triplets& triplets) if
     * you want to preserve the content of triplets.
     *
     * @param triplets triplets containing the non zero elements
     */
    void setFromTriplets(iDynTree::Triplets& triplets);

    static SparseMatrix sparseMatrixFromTriplets(unsigned rows,
                                                 unsigned cols,
                                                 const iDynTree::Triplets& nonZeroElements);


    /**
     * Access operation to the element of the matrix identified by row-col
     *
     * \note this method is slow as it has to look for the proper index (it performs a linear search)
     * @param row row index
     * @param col column index
     * @return the value at the specified row and column
     */
    double operator()(unsigned row, unsigned col) const;

    /**
     * Access operation to the element of the matrix identified by row-col
     *
     * \note this method is slow as it has to look for the proper index (it performs a linear search)
     * \warning if no elements are found, this method insert a zero value at the specified position
     * @param row row index
     * @param col column index
     * @return reference to the value at the specified row and column
     */
    double& operator()(unsigned row, unsigned col);

    inline double getValue(unsigned row, unsigned col) const
    {
        return this->operator()(row, col);
    }

    inline void setValue(unsigned row, unsigned col, double newValue)
    {
        double &value = this->operator()(row, col);
        value = newValue;
    }


    /**
     * Returns the number of rows of the matrix
     * @return the number of rows
     */
    unsigned rows() const;

    /**
     * Returns the number of columns of the matrix
     * @return the number of columns
     */
    unsigned columns() const;

    //Raw buffers access
    double * valuesBuffer();

    double const * valuesBuffer() const;

    int * innerIndicesBuffer();

    int const * innerIndicesBuffer() const;

    int * outerIndicesBuffer();

    int const * outerIndicesBuffer() const;

    // Deprecated
    IDYNTREE_DEPRECATED_WITH_MSG("Use innerIndicesBuffer() instead") int * innerIndecesBuffer();
    IDYNTREE_DEPRECATED_WITH_MSG("Use innerIndicesBuffer() instead") int const * innerIndecesBuffer() const;
    IDYNTREE_DEPRECATED_WITH_MSG("Use outerIndicesBuffer() instead") int * outerIndecesBuffer();
    IDYNTREE_DEPRECATED_WITH_MSG("Use outerIndicesBuffer() instead") int const * outerIndecesBuffer() const;



    /**
     * Returns a textual description of the matrix.
     *
     * If true is passed, the whole matrix (with also zero elements) is printed
     * Default to false
     * @param fullMatrix true to return the full matrix, false for only the non zero elements. Default to false
     * @return a textual representation of the matrix
     */
    std::string description(bool fullMatrix = false) const;

#ifndef NDEBUG
    std::string internalDescription() const;
#endif

    class Iterator;
    class ConstIterator;

    typedef Iterator iterator;
    typedef ConstIterator const_iterator;

    //Why the compiler is not able to choose the const version
    //if we have both? @traversaro
    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    void convertToColumnMajor(double *values, int *innerBuffer, int *outerBuffer) const;
    void convertFromColumnMajor(unsigned rows,
                                unsigned columns,
                                unsigned numberOfNonZeros,
                                double * const values,
                                int * const innerBuffer,
                                int * const outerBuffer);
};

class iDynTree::SparseMatrix::Iterator
{
public:
    class TripletRef {
    private:
        int m_row;
        int m_column;
        double *m_value;

        TripletRef(unsigned row, unsigned column, double *value);
        friend class iDynTree::SparseMatrix::Iterator;

    public:

        inline int row() const { return m_row; }
        inline int column() const { return m_column; }
        inline double& value() { return *m_value; }
        inline double value() const { return *m_value; }

    };

private:
    /**
     * Construct a new iterator
     *
     * @param matrix the sparse matrix to iterate
     * @param valid false if the created iterator is invalid. True by default
     */
    Iterator(iDynTree::SparseMatrix &matrix, bool valid = true);
    friend class iDynTree::SparseMatrix;

    iDynTree::SparseMatrix &m_matrix;

    int m_index;
    TripletRef m_currentTriplet;
    int m_nonZerosInRow;

    void updateTriplet();

public:

    typedef std::ptrdiff_t difference_type;
    typedef iDynTree::SparseMatrix::Iterator::TripletRef value_type;
    typedef value_type& reference;
    typedef value_type* pointer;
    typedef std::output_iterator_tag iterator_category;

    // Required by the iterator type
    Iterator& operator++();
    Iterator operator++(int);

    // Required by the input iterator
    bool operator==(const Iterator&) const;
    bool operator==(const ConstIterator&) const;
    inline bool operator!=(const Iterator& s) const { return !this->operator==(s); }
    inline bool operator!=(const ConstIterator& s) const { return !this->operator==(s); }

    // Required by the input iterator
    // Also Output iterator if the reference modifies the container
    reference operator*();
    pointer operator->();

    bool isValid() const;
};

class iDynTree::SparseMatrix::ConstIterator
{

private:
    /**
     * Construct a new const iterator
     *
     * @param matrix the sparse matrix to iterate
     * @param valid false if the created iterator is invalid. True by default
     */
    ConstIterator(const iDynTree::SparseMatrix &matrix, bool valid = true);
    friend class iDynTree::SparseMatrix;

    const iDynTree::SparseMatrix &m_matrix;

    int m_index;
    iDynTree::Triplet m_currentTriplet;
    int m_nonZerosInRow;

    void updateTriplet();

public:

    typedef std::ptrdiff_t difference_type;
    typedef iDynTree::Triplet value_type;
    typedef const value_type& reference;
    typedef const value_type* pointer;
    typedef std::output_iterator_tag iterator_category;

    //Copy from non const version
    ConstIterator(const Iterator& iterator);

    // Required by the iterator type
    ConstIterator& operator++();
    ConstIterator operator++(int);

    // Required by the input iterator
    bool operator==(const ConstIterator&) const;
    bool operator==(const Iterator&) const;
    inline bool operator!=(const ConstIterator& s) const { return !this->operator==(s); }
    inline bool operator!=(const Iterator& s) const { return !this->operator==(s); }

    // Required by the input iterator
    // Also Output iterator if the reference modifies the container
    reference operator*();
    pointer operator->();

    bool isValid() const;
};



#endif /* end of include guard: IDYNTREE_SPARSE_MATRIX_H */
