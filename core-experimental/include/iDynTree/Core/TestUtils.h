/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TEST_UTILS_H
#define IDYNTREE_TEST_UTILS_H

#include <iDynTree/Core/IVector.h>
#include <iDynTree/Core/IMatrix.h>
#include <iDynTree/Core/Utils.h>

#include <string>

namespace iDynTree
{
    class Transform;

#define ASSERT_EQUAL_DOUBLE(val1,val2) assertDoubleAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_VECTOR(val1,val2) assertVectorAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_MATRIX(val1,val2) assertMatrixAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_TRANSFORM(val1,val2) assertTransformAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)

    void assertDoubleAreEqual(const double & val1, const double & val2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    /**
     * Assert that two vectors are equal, and
     * exit with EXIT_FAILURE if they are not.
     *
     */
    void assertVectorAreEqual(const IVector & vec1, const IVector & vec2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    /**
     * Assert that two matrices are equal, and
     * exit with EXIT_FAILURE if they are not.
     *
     */
    void assertMatrixAreEqual(const IMatrix & mat1, const IMatrix & mat2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    /**
     * Assert that two transforms are equal, and
     * exit with EXIT_FAILURE if they are not.
     *
     */
    void assertTransformsAreEqual(const Transform & trans1, const Transform & trans2, double tol = DEFAULT_TOL, std::string file="", int line=-1);
}

#endif