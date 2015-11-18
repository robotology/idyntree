/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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

    assertVectorAreEqual(dirToNormalize,dirNormalized);

    //test the fact that the construct is normalizing
    Direction dirToNormalize2(1.0,2.0,2.0), dirNormalized2(1.0/3.0,2.0/3.0,2.0/3.0);

    assertVectorAreEqual(dirToNormalize2,dirNormalized2);



    return EXIT_SUCCESS;
}