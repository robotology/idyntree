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
    SparseMatrix matrix(5,5);
    matrix(2,0) = 7;
    std::cout << matrix.description() << "\n";
    matrix(0,1) = 3;
    std::cout << matrix.description() << "\n";
    matrix(1,0) = 22;
    std::cout << matrix.description() << "\n";
    matrix(1,4) = 17;
    std::cout << matrix.description() << "\n";
    matrix(2,1) = 5;
    std::cout << matrix.description() << "\n";
    matrix(2,3) = 1;
    std::cout << matrix.description() << "\n";
    matrix(4,2) = 14;
    std::cout << matrix.description() << "\n";
    matrix(4,4) = 8;
    std::cout << matrix.description() << "\n";

    std::cout << "Matrix:\n" << matrix.description(true) << "\n";
}

void testCreateMatrixFromTriplets()
{
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
}

void testCreateMatrixFromDuplicateTriplets()
{
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
}

void testMatrixIterator(SparseMatrix matrix)
{
    //iterator can modify values
    for (SparseMatrix::iterator it(matrix.begin()); it != matrix.end(); ++it) {
        it->value()++;
    }

    //const iterator cannot
    for (SparseMatrix::const_iterator it(matrix.begin()); it!= matrix.end() ; ++it) {
        std::cout << it->value << "(" << it->row << "," << it->column << ")\n";
    }
}

void testZeroingMatrix(SparseMatrix matrix)
{
    matrix.zero();
    std::cout << matrix.description() << std::endl;
}

void testClassesTraits()
{
#if __cplusplus > 199711L
    std::cout << std::boolalpha;
    std::cout << "is_trivially_destructible:" << std::endl;
    std::cout << "Triplet: " << std::is_trivially_destructible<Triplet>::value << std::endl;
    std::cout << "Triplets: " << std::is_trivially_destructible<Triplets>::value << std::endl;
#endif
}

void testRowColumnMajorConversion(SparseMatrix matrix)
{
    std::cout << "Initial matrix" << std::endl;
    std::cout << matrix.description(true) << std::endl << std::endl;

    double *values = new double[matrix.numberOfNonZeros()];
    int *inner = new int[matrix.numberOfNonZeros()];
    int *outer = new int[matrix.columns() + 1];

    matrix.convertToColumnMajor(values, inner, outer);

    std::cout << "Values and column\n";
    for (unsigned i = 0; i < matrix.numberOfNonZeros(); ++i) {
        std::cout << "(" << values[i] << "," << inner[i] << ") ";
    }
    std::cout << "\nouter\n";
    for (unsigned i = 0; i <= matrix.columns(); ++i) {
        std::cout << outer[i] << " ";
    }

    //Now convert back to row
    SparseMatrix rowMatrix(matrix.rows(), matrix.columns());
    rowMatrix.convertFromColumnMajor(matrix.rows(), matrix.columns(),
                                     matrix.numberOfNonZeros(),
                                     values,
                                     inner,
                                     outer);

    std::cerr << "\n\nRow matrix is\n" << rowMatrix.description(true) << "\n";

    delete [] values;
    delete [] inner;
    delete [] outer;

    //Now I should assert matrix == rowMatrix

    std::flush(std::cout);

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
