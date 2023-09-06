// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// TODO: change this to correct include
#include <iDynTree/OptimalControl.h>

#include <iDynTree/MatrixDynSize.h>

int main()
{
    using namespace iDynTree;
    using namespace iDynTree::optimalcontrol;

    // Linear OC problemm with constraints
    // start with table cart system (constant z)
    // 6 states, 2 control variables
    const double z = 0.15;
    const double g = -9.81;
    LinearSystem tableCart(6, 2, false);
    MatrixDynSize A(6, 6);
    A.zero();
    A(0, 1) = 1;
    A(1, 0) = g / z;
    A(3, 4) = 1;
    A(4, 3) = g / z;
    tableCart.setConstantStateMatrix(A);

    // The Matrix B
    MatrixDynSize B(6, 2);
    B.zero();
    B(2, 0) = 1;
    B(5, 1) = 1;
    tableCart.setConstantControlMatrix(B);


    OptimalControlProblem controlProblem;
    controlProblem.setDynamicalSystemConstraint(tableCart);

    // creating ZMP tracking cost
    //
    MatrixDynSize zmp(2, 6);
    zmp.zero();
    zmp(0, 0) = 1;
    zmp(0, 2) = -z / g;
    zmp(1, 3) = 1;
    zmp(1, 5) = -z / g;

    





    return 0;
}
