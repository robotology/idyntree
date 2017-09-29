/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <cstdlib>

using namespace iDynTree;

void validateSkewUnskew(const Vector3 randVec)
{
    Vector3 randVecCheck;

    toEigen(randVecCheck) = unskew(skew(toEigen(randVec)));

    ASSERT_EQUAL_VECTOR(randVec,randVecCheck);
}

void testSpatialVectors()
{
    SpatialForceVector zeroVector;
    zeroVector.zero();

    for (size_t i = 0; i < zeroVector.size(); ++i) {
        ASSERT_EQUAL_DOUBLE(zeroVector(i), 0);
    }

    // now change the number
    // But as the method is read-only, this should not actually change the initial value
    toEigen(zeroVector).setConstant(3.14);
    for (size_t i = 0; i < zeroVector.size(); ++i) {
        ASSERT_EQUAL_DOUBLE(zeroVector(i), 0);
    }

}

int main()
{
    Vector3 vec;

    vec(0) = 1;
    vec(1) = 2;
    vec(2) = 3;

    validateSkewUnskew(vec);

    testSpatialVectors();


    return EXIT_SUCCESS;
}
