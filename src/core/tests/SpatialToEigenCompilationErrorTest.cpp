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
