// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/BerdySparseMAPSolver.h>

#include <iDynTree/BerdyHelper.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenSparseHelpers.h>
#include <iDynTree/TestUtils.h>

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
