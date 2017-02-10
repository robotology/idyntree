/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/TestUtils.h>
#include <iostream>


using namespace iDynTree;
using namespace std;

void testCreateMatrixFromAccessorOperator()
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromAccessorOperator" << std::endl;
    SparseMatrix matrix(5,5);
    matrix(2,0) = 7;
    std::cout << matrix.description() << std::endl;
    matrix(0,1) = 3;
    std::cout << matrix.description() << std::endl;
    matrix(1,0) = 22;
    std::cout << matrix.description() << std::endl;
    matrix(1,4) = 17;
    std::cout << matrix.description() << std::endl;
    matrix(2,1) = 5;
    std::cout << matrix.description() << std::endl;
    matrix(2,3) = 1;
    std::cout << matrix.description() << std::endl;
    matrix(4,2) = 14;
    std::cout << matrix.description() << std::endl;
    matrix(4,4) = 8;
    std::cout << matrix.description() << std::endl;

    std::cout << "Matrix:\n" << matrix.description(true) << std::endl;
    std::cout << "------------------------------------" << std::endl;
}

void testCreateMatrixFromTriplets()
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromTriplets" << std::endl;
    SparseMatrix matrix(5,5);
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

    std::cout << "Matrix:\n" << matrix.description(true) << "\n";
    std::cout << "------------------------------------" << std::endl;
}

void testCreateMatrixFromDuplicateTriplets()
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testCreateMatrixFromDuplicateTriplets" << std::endl;
    SparseMatrix matrix(5,5);
    Triplets triplets;
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
}

void testMatrixIterator(SparseMatrix matrix)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testMatrixIterator" << std::endl;
    //iterator can modify values
    for (SparseMatrix::iterator it(matrix.begin()); it != matrix.end(); ++it) {
        it->value()++;
    }

    //const iterator cannot
    for (SparseMatrix::const_iterator it(matrix.begin()); it!= matrix.end() ; ++it) {
        std::cout << it->value << "(" << it->row << "," << it->column << ")\n";
    }

    std::cout << "------------------------------------" << std::endl;
}

void testZeroingMatrix(SparseMatrix matrix)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testZeroingMatrix" << std::endl;
    matrix.zero();
    std::cout << matrix.description() << std::endl;
    std::cout << "------------------------------------" << std::endl;
}

void testClassesTraits()
{
#if __cplusplus > 199711L
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testClassesTraits" << std::endl;
    std::cout << std::boolalpha;
    std::cout << "is_trivially_destructible:" << std::endl;
    std::cout << "Triplet: " << std::is_trivially_destructible<Triplet>::value << std::endl;
    std::cout << "Triplets: " << std::is_trivially_destructible<Triplets>::value << std::endl;
    std::cout << "------------------------------------" << std::endl;
#endif
}

void testRowColumnMajorConversion(SparseMatrix matrix)
{
    std::cout << "------------------------------------" << std::endl;
    std::cout << "testRowColumnMajorConversion" << std::endl;
    std::cout << "Initial (row Major) matrix" << std::endl;
    std::cout << matrix.description(true) << std::endl << std::endl;

    double *values = new double[matrix.numberOfNonZeros()];
    int *inner = new int[matrix.numberOfNonZeros()];
    int *outer = new int[matrix.columns() + 1];

    matrix.convertToColumnMajor(values, inner, outer);

    std::cout << "Col Major matrix" << std::endl;
    std::cout << "Values and column" << std::endl;
    for (unsigned i = 0; i < matrix.numberOfNonZeros(); ++i) {
        std::cout << "(" << values[i] << "," << inner[i] << ") ";
    }
    std::cout << std::endl << "outer" << std::endl;
    for (unsigned i = 0; i <= matrix.columns(); ++i) {
        std::cout << outer[i] << " ";
    }
    std::cout << std::endl;

    //Now convert back to row
    SparseMatrix rowMatrix(matrix.rows(), matrix.columns());
    rowMatrix.convertFromColumnMajor(matrix.rows(), matrix.columns(),
                                     matrix.numberOfNonZeros(),
                                     values,
                                     inner,
                                     outer);

    std::cout << "Row Major matrix (converted)" << std::endl;
    std::cout << rowMatrix.description(true) << std::endl;

    delete [] values;
    delete [] inner;
    delete [] outer;

    //Now I should assert matrix == rowMatrix

    std::cout << "------------------------------------" << std::endl;
}

int main()
{
    testCreateMatrixFromAccessorOperator();
    testCreateMatrixFromTriplets();
    testCreateMatrixFromDuplicateTriplets();

    SparseMatrix matrix(5,5);
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
    testRowColumnMajorConversion(matrix);

    SparseMatrix matrix2(5,5);
    matrix2.setValue(0, 2, 5);
    matrix2.setValue(1, 4, 8.8);
    matrix2.setValue(2, 2, 1.8);

    std::cerr << "\n\n";
    testRowColumnMajorConversion(matrix2);
}
