/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
