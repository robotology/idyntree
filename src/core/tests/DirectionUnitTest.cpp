/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>

#include <cstdlib>

using namespace iDynTree;

int main()
{
    // test normalization
    Direction dirToNormalize, dirNormalized;

    // we use the pythagorean quadruple 1,2,2,3
    dirToNormalize(0) = 1;
    dirToNormalize(1) = 2;
    dirToNormalize(2) = 2;

    dirNormalized(0) = 1.0/3.0;
    dirNormalized(1) = 2.0/3.0;
    dirNormalized(2) = 2.0/3.0;

    dirToNormalize.Normalize();

    ASSERT_EQUAL_VECTOR(dirToNormalize,dirNormalized);

    //test the fact that the construct is normalizing
    Direction dirToNormalize2(1.0,2.0,2.0), dirNormalized2(1.0/3.0,2.0/3.0,2.0/3.0);

    ASSERT_EQUAL_VECTOR(dirToNormalize2,dirNormalized2);



    return EXIT_SUCCESS;
}