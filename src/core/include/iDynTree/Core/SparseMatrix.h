/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SPARSE_MATRIX_H
#define IDYNTREE_SPARSE_MATRIX_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/Utils.h>

#include <vector>

namespace iDynTree {

    template <MatrixStorageOrdering ordering>
    class SparseMatrix;

    class Triplet;
    class Triplets;
}

// MARK: - SparseMatrix class

/**
 * \brief Sparse Matrix class
 *
 * This class uses the Compressed Column (Row) Storage scheme
 * (see https://en.wikipedia.org/wiki/Sparse_matrix#Compressed_sparse_row_.28CSR.2C_CRS_or_Yale_format.29)
 * which is compatible with the format used in the Eigen library (by using Map).
 */
template <iDynTree::MatrixStorageOrdering ordering>
class iDynTree::SparseMatrix
{
private:

    iDynTree::VectorDynSize m_values; /**< Contains all the non zero (NZ) elements */

    std::vector<int> m_innerIndices; /**< column index of the NZ elements  */
    std::vector<int> m_outerStarts; /**< for each row contains the index of the first NZ in the
                                     *   previous two arrays
                                     */


    std::size_t m_allocatedSize; /**< size of the memory allocated for m_values and m_innerIndices */

    std::size_t m_rows;
    std::size_t m_columns;

    void initializeMatrix(std::size_t outerSize, const double* vector, std::size_t vectorSize);

    /**
     * Insert a new element at the specified position
     *
     * It returns the index in m_values of the inserted element
     * @param row row of the new element
     * @param col column of the new element
     * @param value value to be inserted
     * @return index in m_values of the inserted element
     */
    std::size_t insert(std::size_t row, std::size_t col, double value);

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
    bool valueIndexForOuterAndInnerIndices(std::size_t outerIndex, std::size_t innerIndex, std::size_t& valueIndex) const;

public:

    /**
     * Creates an empty sparse matrix.
     */
    SparseMatrix();

    /**
     * Creates a zero sparse matrix with the specified dimensions
     *
     */
    SparseMatrix(std::size_t rows, std::size_t cols);


    SparseMatrix(std::size_t rows, std::size_t cols,
                 const iDynTree::VectorDynSize& memoryReserveDescription);

    template <iDynTree::MatrixStorageOrdering otherOrdering>
    SparseMatrix(const SparseMatrix<otherOrdering>&);

    template <iDynTree::MatrixStorageOrdering otherOrdering>
    SparseMatrix& operator=(const SparseMatrix<otherOrdering>&);


    /**
     * Default destructor
     */
    ~SparseMatrix();

    /**
     * Returns the number of nonzero elements in this sparse matrix
     *
     * @return the number of non zero elements
     */
    std::size_t numberOfNonZeros() const;

    /**
     * Resize the matrix to the specified new dimensions
     *
     * \note the content of the matrix is not preserved in any case
     * \warning this function may perform memory allocation
     * @param rows the new number of rows of this matrix
     * @param columns the new number of columns of this matrix
     */
    void resize(std::size_t rows, std::size_t columns);

    /**
     * Resize the matrix to the specified new dimensions
     *
     * \note the content of the matrix is not preserved in any case
     * \warning this function may perform memory allocation
     * @param rows the new number of rows of this matrix
     * @param columns the new number of columns of this matrix
     * @param innerIndicesInformation information on the NNZ for each column (row), used to reserve memory in advance. It depends on the storage ordering
     */
    void resize(std::size_t rows, std::size_t columns, const iDynTree::VectorDynSize &innerIndicesInformation);

    void reserve(std::size_t nonZeroElements);


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

    static SparseMatrix sparseMatrixFromTriplets(std::size_t rows,
                                                 std::size_t cols,
                                                 const iDynTree::Triplets& nonZeroElements);


    /**
     * Access operation to the element of the matrix identified by row-col
     *
     * \note this method is slow as it has to look for the proper index (it performs a linear search)
     * @param row row index
     * @param col column index
     * @return the value at the specified row and column
     */
    double operator()(std::size_t row, std::size_t col) const;

    /**
     * Access operation to the element of the matrix identified by row-col
     *
     * \note this method is slow as it has to look for the proper index (it performs a linear search)
     * \warning if no elements are found, this method insert a zero value at the specified position
     * @param row row index
     * @param col column index
     * @return reference to the value at the specified row and column
     */
    double& operator()(std::size_t row, std::size_t col);

    inline double getValue(std::size_t row, std::size_t col) const
    {
        return this->operator()(row, col);
    }

    inline void setValue(std::size_t row, std::size_t col, double newValue)
    {
        double &value = this->operator()(row, col);
        value = newValue;
    }


    /**
     * Returns the number of rows of the matrix
     * @return the number of rows
     */
    std::size_t rows() const;

    /**
     * Returns the number of columns of the matrix
     * @return the number of columns
     */
    std::size_t columns() const;

    //Raw buffers access
    double * valuesBuffer();

    double const * valuesBuffer() const;

    int * innerIndicesBuffer();

    int const * innerIndicesBuffer() const;

    int * outerIndicesBuffer();

    int const * outerIndicesBuffer() const;


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

};

// MARK: - Iterator classes

#ifndef SWIG

template <iDynTree::MatrixStorageOrdering ordering>
class iDynTree::SparseMatrix<ordering>::Iterator
{
public:
    class TripletRef {
    private:
        int m_row;
        int m_column;
        double *m_value;

        TripletRef(std::size_t row, std::size_t column, double *value);
        friend class iDynTree::SparseMatrix<ordering>::Iterator;

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
    Iterator(iDynTree::SparseMatrix<ordering> &matrix, bool valid = true);
    friend class iDynTree::SparseMatrix<ordering>;

    iDynTree::SparseMatrix<ordering> &m_matrix;

    int m_index;
    TripletRef m_currentTriplet;
    int m_nonZerosInOuterDirection;

    void updateTriplet();

public:

    typedef std::ptrdiff_t difference_type;
    typedef typename iDynTree::SparseMatrix<ordering>::Iterator::TripletRef value_type;
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

template <iDynTree::MatrixStorageOrdering ordering>
class iDynTree::SparseMatrix<ordering>::ConstIterator
{

private:
    /**
     * Construct a new const iterator
     *
     * @param matrix the sparse matrix to iterate
     * @param valid false if the created iterator is invalid. True by default
     */
    ConstIterator(const iDynTree::SparseMatrix<ordering> &matrix, bool valid = true);
    friend class iDynTree::SparseMatrix<ordering>;

    const iDynTree::SparseMatrix<ordering> &m_matrix;

    int m_index;
    iDynTree::Triplet m_currentTriplet;
    int m_nonZerosInOuterDirection;

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

#endif

#endif /* end of include guard: IDYNTREE_SPARSE_MATRIX_H */
