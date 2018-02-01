/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/BerdySparseMAPSolver.h>

#include <iDynTree/Estimation/BerdyHelper.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/TestUtils.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void testEmptyHelper()
{
    BerdyHelper helper;
    BerdySparseMAPSolver solver(helper);

    ASSERT_IS_FALSE(solver.isValid());
}

int main()
{
    testEmptyHelper();

    return EXIT_SUCCESS;
}
