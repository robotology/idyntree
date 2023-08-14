// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
