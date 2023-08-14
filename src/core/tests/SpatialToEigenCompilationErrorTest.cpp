// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <cstdlib>

using namespace iDynTree;

int main()
{
    // The toEigen method for spatial vector is read-only, so this should result in a compilation error
    // See https://github.com/robotology/idyntree/pull/378
    Twist zeroVector;
    toEigen(zeroVector).setConstant(3.14);

    Wrench zeroVectorW;
    toEigen(zeroVectorW).setConstant(3.14);

    return EXIT_SUCCESS;
}
