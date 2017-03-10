/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/InverseKinematics.h>
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include "testModels.h"

#include <cstdio>
#include <cstdlib>

#include <ctime>

/**
 * Return the current time in seconds, with respect
 * to an arbitrary point in time.
 */
inline double clockInSec()
{
    clock_t ret = clock();
    return ((double)ret)/((double)CLOCKS_PER_SEC);
}


iDynTree::JointPosDoubleArray getRandomJointPositions(const iDynTree::Model & model)
{
    iDynTree::JointPosDoubleArray sRandom(model);
    getRandomJointPositions(sRandom,model);
    return sRandom;
}

// Check the consistency of a simple IK test case
void simpleIKConsistency()
{
    iDynTree::InverseKinematics ik;

    bool ok = ik.loadModelFromFile(getAbsModelPath("iCubGenova02.urdf"));
    ASSERT_IS_TRUE(ok);

    // For now, just use euler angles to represent rotations
    ik.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    // Create also a KinDyn object to perform forward kinematics for the desired values
    iDynTree::KinDynComputations kinDynDes;
    ok = kinDynDes.loadRobotModel(ik.model());
    ASSERT_IS_TRUE(ok);

    // Create a random vector of internal joint positions
    // used to get reasonable values for the constraints and the targets
    iDynTree::JointPosDoubleArray s = getRandomJointPositions(kinDynDes.model());
    ok = kinDynDes.setJointPos(s);
    ASSERT_IS_TRUE(ok);

    // Create a simple IK problem with the foot constraint

    // The l_sole frame is our world absolute frame (i.e. {}^A H_{l_sole} = identity
    ok = ik.addFrameConstraint("l_sole",iDynTree::Transform::Identity());
    ASSERT_IS_TRUE(ok);

    // The relative position of the r_sole should be the initial one
    //ok = ik.addFrameConstraint("r_sole",kinDynDes.getRelativeTransform("l_sole","r_sole"));
    //ASSERT_IS_TRUE(ok);

    // The two cartesian targets should be reasonable values
    ok = ik.addPositionTarget("l_elbow_1",kinDynDes.getRelativeTransform("l_sole","l_elbow_1").getPosition());
    ASSERT_IS_TRUE(ok);

    //ok = ik.addPositionTarget("r_elbow_1",kinDynDes.getRelativeTransform("l_sole","r_elbow_1").getPosition());
    //ASSERT_IS_TRUE(ok);

    iDynTree::Transform initialH = kinDynDes.getRelativeTransform("l_sole","root_link");
    ik.setInitialCondition(&initialH,&s);
    ik.setDesiredJointConfiguration(s);

    std::cerr << "Initial H: " << initialH.toString() << std::endl;
    std::cerr << "s " << s.toString() << std::endl;


    // Solve the optimization problem
    double tic = clockInSec();
    ok = ik.solve();
    double toc = clockInSec();
    std::cerr << "Inverse Kinematics solved in " << toc-tic << " seconds. " << std::endl;
    ASSERT_IS_TRUE(ok);

    iDynTree::JointPosDoubleArray sOptimized(ik.model());
    sOptimized.zero();

    // We create a new KinDyn object to perform forward kinematics for the optimized values
    iDynTree::KinDynComputations kinDynOpt;
    kinDynOpt.loadRobotModel(ik.model());
    kinDynOpt.setJointPos(sOptimized);

    // Check that the contraint and the targets are respected
    double tolConstraints = 1e-5;
    double tolTargets     = 1e-4;
    ASSERT_EQUAL_TRANSFORM(kinDynDes.getRelativeTransform("l_sole","r_sole"),kinDynOpt.getRelativeTransform("l_sole","r_sole"));

    ASSERT_EQUAL_VECTOR(kinDynDes.getRelativeTransform("l_sole","r_elbow_1").getPosition(),
                        kinDynOpt.getRelativeTransform("l_sole","r_elbow_1").getPosition());
    ASSERT_EQUAL_VECTOR(kinDynDes.getRelativeTransform("l_sole","l_elbow_1").getPosition(),
                        kinDynOpt.getRelativeTransform("l_sole","l_elbow_1").getPosition());

    return;
}

int main()
{

    simpleIKConsistency();
    return EXIT_SUCCESS;
}
