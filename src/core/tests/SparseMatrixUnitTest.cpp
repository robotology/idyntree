// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SparseMatrix.h>
#include <iDynTree/Triplets.h>
#include <iDynTree/TestUtils.h>
#include <iostream>


using namespace iDynTree;
using namespace std;

template <iDynTree::MatrixStorageOrdering ordering>
void testCreateMatrixFromAccessorOperator()
{

    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromAccessorOperator" << std::endl;

    Triplets triplets;
    triplets.pushTriplet(iDynTree::Triplet(2, 0, 7));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 3));
    triplets.pushTriplet(iDynTree::Triplet(1, 0, 22));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, 17));
    triplets.pushTriplet(iDynTree::Triplet(2, 1, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 3, 1));
    triplets.pushTriplet(iDynTree::Triplet(4, 2, 14));
    triplets.pushTriplet(iDynTree::Triplet(4, 4, 8));

    SparseMatrix<ordering> matrix(5, 5);
    for (Triplets::const_iterator it(triplets.begin()); it != triplets.end(); ++it) {
        matrix(it->row, it->column) = it->value;
    }

    std::cout << "Matrix:\n" << matrix.description(true) << std::endl;
    std::cout << "------------------------------------" << std::endl;

    //expect elements at the correct position
    for (Triplets::const_iterator it(triplets.begin()); it != triplets.end(); ++it) {
        ASSERT_EQUAL_DOUBLE(matrix(it->row, it->column), it->value);
        matrix(it->row, it->column) = it->value;
    }

}

template <iDynTree::MatrixStorageOrdering ordering>
void testCreateMatrixFromTriplets()
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromTriplets" << std::endl;
    SparseMatrix<ordering> matrix;
    matrix.resize(5, 5);
    Triplets triplets;
    triplets.pushTriplet(iDynTree::Triplet(0, 0, -6));
    triplets.pushTriplet(iDynTree::Triplet(2, 0, 7));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 3));
    triplets.pushTriplet(iDynTree::Triplet(1, 0, 22));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, 17));
    triplets.pushTriplet(iDynTree::Triplet(2, 1, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 3, 1));
    triplets.pushTriplet(iDynTree::Triplet(4, 2, 14));
    triplets.pushTriplet(iDynTree::Triplet(4, 4, 8));

    matrix.setFromTriplets(triplets);

    std::cout << "Matrix:\n" << matrix.description(true) << "\n";
    std::cout << "------------------------------------" << std::endl;

    //expect elements at the correct position
    for (Triplets::const_iterator it(triplets.begin()); it != triplets.end(); ++it) {
        ASSERT_EQUAL_DOUBLE(matrix(it->row, it->column), it->value);
        matrix(it->row, it->column) = it->value;
    }
}

template <iDynTree::MatrixStorageOrdering ordering>
void testCreateMatrixFromDuplicateTriplets()
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromDuplicateTriplets" << std::endl;
    SparseMatrix<ordering> matrix(5, 5);
    Triplets triplets;
    triplets.pushTriplet(iDynTree::Triplet(0, 0, -6));
    triplets.pushTriplet(iDynTree::Triplet(0, 0, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 0, 7));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 3));
    triplets.pushTriplet(iDynTree::Triplet(1, 0, 22));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, 17));
    triplets.pushTriplet(iDynTree::Triplet(2, 1, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 3, 1));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, -5));
    triplets.pushTriplet(iDynTree::Triplet(4, 2, 14));
    triplets.pushTriplet(iDynTree::Triplet(4, 4, 8));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 4));

    matrix.setFromTriplets(triplets);

    std::cout << "Matrix:\n" << matrix.description(true) << "\n";
    std::cout << "------------------------------------" << std::endl;

    //I still have to implement a sort of "Merge" triplets.
    //Until that, manually write the asserts

    ASSERT_EQUAL_DOUBLE(matrix(0, 0), -1);
    ASSERT_EQUAL_DOUBLE(matrix(2, 0), 7);
    ASSERT_EQUAL_DOUBLE(matrix(0, 1), 7);
    ASSERT_EQUAL_DOUBLE(matrix(1, 0), 22);
    ASSERT_EQUAL_DOUBLE(matrix(1, 4), 12);
    ASSERT_EQUAL_DOUBLE(matrix(2, 1), 5);
    ASSERT_EQUAL_DOUBLE(matrix(2, 3), 1);
    ASSERT_EQUAL_DOUBLE(matrix(4, 2), 14);
    ASSERT_EQUAL_DOUBLE(matrix(4, 4), 8);

}

template <iDynTree::MatrixStorageOrdering ordering>
void testMatrixIterator(SparseMatrix<ordering> matrix)
{
    SparseMatrix<ordering> originalMatrix(matrix);
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testMatrixIterator" << std::endl;
    //iterator can modify values
    for (typename SparseMatrix<ordering>::iterator it(matrix.begin()); it != matrix.end(); ++it) {
        it->value()++;
    }

    //const iterator cannot
    for (typename SparseMatrix<ordering>::const_iterator it(matrix.begin()); it!= matrix.end() ; ++it) {
        std::cout << it->value << "(" << it->row << "," << it->column << ")\n";
    }

    //Assertion:
    //All the elements in matrix = originalMatrix + 1
    for (typename SparseMatrix<ordering>::const_iterator it(matrix.begin()); it!= matrix.end() ; ++it) {
        ASSERT_EQUAL_DOUBLE(matrix(it->row, it->column), 1 + originalMatrix(it->row, it->column));
    }

    std::cout << "------------------------------------" << std::endl;
}

template <iDynTree::MatrixStorageOrdering ordering>
void testZeroingMatrix(SparseMatrix<ordering> matrix)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testZeroingMatrix" << std::endl;
    matrix.zero();
    std::cout << matrix.description() << std::endl;
    std::cout << "------------------------------------" << std::endl;

    ASSERT_IS_TRUE(matrix.numberOfNonZeros() == 0);
    ASSERT_IS_FALSE(matrix.begin().isValid());

}

void testClassesTraits()
{
#if __cplusplus > 199711L
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testClassesTraits" << std::endl;
    std::cout << std::boolalpha;
    std::cout << "is_trivially_destructible:" << std::endl;
    std::cout << "Triplet: " << std::is_trivially_destructible<Triplet>::value << std::endl;
    ASSERT_IS_TRUE(std::is_trivially_destructible<Triplet>::value);
    std::cout << "Triplets: " << std::is_trivially_destructible<Triplets>::value << std::endl;
    std::cout << "------------------------------------" << std::endl;
#endif
}

void testRowColumnMajorConversion(SparseMatrix<iDynTree::RowMajor> matrix)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testRowColumnMajorConversion" << std::endl;
    std::cout << "Initial (row Major) matrix" << std::endl;
    std::cout << matrix.description(true) << std::endl << std::endl;

    SparseMatrix<iDynTree::ColumnMajor> columnMajorMatrix;
    columnMajorMatrix = matrix;

    for (unsigned row = 0; row < matrix.rows(); row++) {
        for (unsigned col = 0; col < matrix.columns(); col++) {
            ASSERT_EQUAL_DOUBLE(matrix(row, col), columnMajorMatrix(row, col));
        }
    }

    SparseMatrix<iDynTree::RowMajor> rowMajorMatrixConverted(columnMajorMatrix);

    //Now I should assert matrix == rowMatrix
    //Do the long computation: all all the values!
    for (unsigned row = 0; row < matrix.rows(); row++) {
        for (unsigned col = 0; col < matrix.columns(); col++) {
            ASSERT_EQUAL_DOUBLE(matrix(row, col), rowMajorMatrixConverted(row, col));
        }
    }

    std::cout << "------------------------------------" << std::endl;
}

int main()
{
    std::cerr << "Testing RowMajor ordering" << std::endl;
    // Row major
    testCreateMatrixFromAccessorOperator<iDynTree::RowMajor>();
    testCreateMatrixFromTriplets<iDynTree::RowMajor>();
    testCreateMatrixFromDuplicateTriplets<iDynTree::RowMajor>();

    SparseMatrix<iDynTree::RowMajor> matrix(5, 5);
    Triplets triplets;
    triplets.pushTriplet(iDynTree::Triplet(2, 0, 7));
    triplets.pushTriplet(iDynTree::Triplet(0, 1, 3));
    triplets.pushTriplet(iDynTree::Triplet(1, 0, 22));
    triplets.pushTriplet(iDynTree::Triplet(1, 4, 17));
    triplets.pushTriplet(iDynTree::Triplet(2, 1, 5));
    triplets.pushTriplet(iDynTree::Triplet(2, 3, 1));
    triplets.pushTriplet(iDynTree::Triplet(4, 2, 14));
    triplets.pushTriplet(iDynTree::Triplet(4, 4, 8));

    matrix.setFromTriplets(triplets);

    testMatrixIterator(matrix);
    testZeroingMatrix(matrix);
    testClassesTraits();

    std::cerr << "Testing ColumnMajor ordering" << std::endl;
    //Column major
    testCreateMatrixFromAccessorOperator<iDynTree::ColumnMajor>();
    testCreateMatrixFromTriplets<iDynTree::ColumnMajor>();
    testCreateMatrixFromDuplicateTriplets<iDynTree::ColumnMajor>();


    std::cerr << "Testing RowMajor-ColumnMajor conversions" << std::endl;
    // Testing conversion

    testRowColumnMajorConversion(matrix);

    SparseMatrix<iDynTree::RowMajor> matrix2(5, 5);
    matrix2.setValue(0, 2, 5);
    matrix2.setValue(1, 4, 8.8);
    matrix2.setValue(2, 2, 1.8);

    std::cerr << "\n\n";
    testRowColumnMajorConversion(matrix2);




}
