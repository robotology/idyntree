/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/iKinConversions.h>

#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/JointState.h>

#include <iDynTree/yarp/YARPConversions.h>

#include <iCub/iKin/iKinFwd.h>

#include <iDynTree/Model/DenavitHartenberg.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void check_iKinImportFromChain(iCub::iKin::iKinChain & chain)
{
    // First unblock all DOF of the chain
    for (int i=0; i < chain.getN(); i++)
    {
        chain.releaseLink(i);
    }


    Model model;
    bool ok = modelFromiKinChain(chain, model);
    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_DOUBLE(chain.getDOF(), model.getNrOfDOFs());

    KinDynComputations dynComp;
    ok = dynComp.loadRobotModel(model);
    ASSERT_IS_TRUE(ok);

    // Given how the model is built by modelFromiKinChain, we now that link 0 is the base
    // and the one with the highest index is the ee
    Vector3 grav;
    grav.zero();
    JointPosDoubleArray qj(model);
    JointDOFsDoubleArray dqj(model);
    dqj.zero();

    yarp::sig::Vector qj_yarp(model.getNrOfPosCoords());
    yarp::sig::Vector dqj_yarp(model.getNrOfDOFs());

    // Check for some various positions
    for (int i=0; i < 10; i++ )
    {
        getRandomVector(qj, -3.14, 3.14);
        toYarp(qj, qj_yarp);
        toYarp(dqj, dqj_yarp);

        // First set the position in iKin to check any limit
        qj_yarp = chain.setAng(qj_yarp);
        toiDynTree(qj_yarp, qj);

        dynComp.setRobotState(qj, dqj, grav);

        // Get b_H_ee for both iKin and iDynTree
        LinkIndex baseIndex = model.getFrameIndex("baseFrame");
        LinkIndex eeIndex   = model.getFrameIndex("distalFrame");
        Transform b_H_ee = dynComp.getRelativeTransform(baseIndex, eeIndex);
        yarp::sig::Matrix b_H_ee_ikin_yarp = chain.getH();
        Transform b_H_ee_ikin;
        toiDynTree(b_H_ee_ikin_yarp, b_H_ee_ikin);


        ASSERT_EQUAL_TRANSFORM(b_H_ee, b_H_ee_ikin);
    }
}

void check_iKinImportFromChain(DHChain & chain)
{
    iCub::iKin::iKinLimb ikinLimb;
    iKinLimbFromDHChain(chain, ikinLimb);
    check_iKinImportFromChain(ikinLimb);
}

void check_iKinImport()
{
    // Check some simple chains
    std::cerr << "Testing iKin import of 1 dofs chains" << std::endl;
    DHChain dh1dof;

    dh1dof.setH0(Transform::Identity());
    dh1dof.setHN(Transform::Identity());
    dh1dof.setNrOfDOFs(1);
    dh1dof.setDOFName(0, "joint0");
    dh1dof(0).A = dh1dof(0).D = dh1dof(0).Alpha = dh1dof(0).Offset = 0.0;
    dh1dof.setHN(iDynTree::Transform(iDynTree::Rotation::Identity(), iDynTree::Position(0.0, 0.0, 1.0)));
    check_iKinImportFromChain(dh1dof);

    dh1dof(0).A = 1.0;
    check_iKinImportFromChain(dh1dof);
    dh1dof(0).A = 0.0;

    dh1dof(0).D = 1.0;
    check_iKinImportFromChain(dh1dof);
    dh1dof(0).D = 0.0;

    dh1dof(0).Alpha = 1.0;
    check_iKinImportFromChain(dh1dof);
    dh1dof(0).Alpha = 0.0;

    dh1dof(0).Offset = 1.0;
    check_iKinImportFromChain(dh1dof);
    dh1dof(0).Offset = 0.0;

    dh1dof(0).A = dh1dof(0).D = dh1dof(0).Alpha = dh1dof(0).Offset = 1.0;
    check_iKinImportFromChain(dh1dof);
    dh1dof(0).A = dh1dof(0).D = dh1dof(0).Alpha = dh1dof(0).Offset = 0.0;


    // Simple 2 dofs
    std::cerr << "Testing iKin import of 2 dofs chains" << std::endl;
    DHChain dh2dof;

    dh2dof.setH0(Transform::Identity());
    dh2dof.setHN(Transform::Identity());
    dh2dof.setNrOfDOFs(2);
    dh2dof.setDOFName(0, "joint0");
    dh2dof.setDOFName(1, "joint1");

    dh2dof(0).A = dh2dof(0).D = dh2dof(0).Alpha = dh2dof(0).Offset = 0.0;
    dh2dof(1).A = dh2dof(1).D = dh2dof(1).Alpha = dh2dof(1).Offset = 0.0;
    check_iKinImportFromChain(dh2dof);

    dh2dof(0).A = dh2dof(1).A = 1.0;
    check_iKinImportFromChain(dh2dof);
    dh2dof(0).A = dh2dof(1).A = 0.0;

    dh2dof(0).D = dh2dof(1).D = 1.0;
    check_iKinImportFromChain(dh2dof);
    dh2dof(0).D = dh2dof(1).D  = 0.0;

    dh2dof(0).Alpha = dh2dof(1).Alpha  = 1.0;
    check_iKinImportFromChain(dh2dof);
    dh2dof(0).Alpha = dh2dof(1).Alpha  = 0.0;

    dh2dof(0).Offset = dh2dof(1).Offset  = 1.0;
    check_iKinImportFromChain(dh2dof);
    dh2dof(0).Offset = dh2dof(1).Offset  = 0.0;

    dh2dof(0).A = dh2dof(0).D = dh2dof(0).Alpha = dh2dof(0).Offset = 1.0;
    dh2dof(1).A = dh2dof(1).D = dh2dof(1).Alpha = dh2dof(1).Offset = 1.0;
    check_iKinImportFromChain(dh2dof);
    dh2dof(0).A = dh2dof(0).D = dh2dof(0).Alpha = dh2dof(0).Offset = 0.0;
    dh2dof(1).A = dh2dof(1).D = dh2dof(1).Alpha = dh2dof(1).Offset = 0.0;

    // Check iKin hardcoded chains
    std::cerr << "Testing iKin import of iCub chains" << std::endl;

    iCub::iKin::iCubLeg leg;
    check_iKinImportFromChain(leg);
    iCub::iKin::iCubArm arm;
    check_iKinImportFromChain(arm);
}


int main()
{
    check_iKinImport();
    return EXIT_SUCCESS;
}
