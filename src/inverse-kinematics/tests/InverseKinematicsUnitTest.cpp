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

struct simpleChainIKOptions
{
    iDynTree::InverseKinematicsRotationParametrization rotationParametrization;
    bool useDesiredJointPositionsToActualValue;
    bool useDesiredJointPositionsToRandomValue;
};

void simpleChainIK(int minNrOfJoints, int maxNrOfJoints, const iDynTree::InverseKinematicsRotationParametrization rotationParametrization)
{
    // Solve a simple IK problem for a chain, with no constraints 
    for (int i = minNrOfJoints; i <= maxNrOfJoints; i++)
    {
        assert(i >= 2);
        
        std::cerr << "~~~~~~~> simpleChainIK with " << i << " dofs " << std::endl;
        bool noFixedJoints = true;
        iDynTree::Model chain = iDynTree::getRandomChain(i,10,noFixedJoints);

        ASSERT_EQUAL_DOUBLE(i, chain.getNrOfDOFs());
        
        // Name of the targetFrame the leaf added by getRandomChain 
        std::string targetFrame = "link" + iDynTree::int2string(i - 1);

        // Create IK 
        iDynTree::InverseKinematics ik;
        bool ok = ik.setModel(chain);
        ASSERT_IS_TRUE(ok);
        // Always express the target as cost
        ik.setTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);

        // Use the requested parametrization
        ik.setRotationParametrization(rotationParametrization);

        // Create also a KinDyn object to perform forward kinematics for the desired values and the optimized ones
        iDynTree::KinDynComputations kinDynDes;
        ok = kinDynDes.loadRobotModel(ik.model());
        ASSERT_IS_TRUE(ok);
        iDynTree::KinDynComputations kinDynOpt;
        ok = kinDynOpt.loadRobotModel(ik.model());
        ASSERT_IS_TRUE(ok);

        // Create a random vector of internal joint positions
        // used to get reasonable values for the constraints and the targets
        iDynTree::JointPosDoubleArray s = getRandomJointPositions(kinDynDes.model());
        ok = kinDynDes.setJointPos(s);
        ASSERT_IS_TRUE(ok);

        // Add the fixed base constraint 
        ok = ik.addFrameConstraint("link1", kinDynDes.getWorldTransform("link1"));
        //ok = ik.addFrameRotationConstraint("link1", iDynTree::Transform::Identity().getRotation());
        //ok = ik.addFramePositionConstraint("link1", iDynTree::Transform::Identity().getPosition());
        ASSERT_IS_TRUE(ok);

        // Add target 
        ok = ik.addPositionTarget(targetFrame, kinDynDes.getWorldTransform(targetFrame));
        ASSERT_IS_TRUE(ok);
       

        // Add a random initial guess
        iDynTree::Transform baseDes = kinDynDes.getWorldBaseTransform();
        iDynTree::JointPosDoubleArray sInitial = getRandomJointPositions(ik.model());
        ik.setInitialCondition(&(baseDes),&(sInitial));

        // Add a random desired position
        iDynTree::JointPosDoubleArray sDesired = getRandomJointPositions(ik.model());
        ik.setDesiredJointConfiguration(sDesired);

        // Start from the initial one
        double tic = clockInSec();
        ok = ik.solve();
        double toc = clockInSec();
        
        ASSERT_IS_TRUE(ok);
        
        // Get the solution 
        iDynTree::Transform baseOpt = iDynTree::Transform::Identity();
        iDynTree::JointPosDoubleArray sOpt(ik.model());
        ik.getSolution(baseOpt,sOpt);
        iDynTree::Twist dummyVel;
        dummyVel.zero();
        iDynTree::Vector3 dummyGrav;
        dummyGrav.zero();
        iDynTree::JointDOFsDoubleArray dummyJointVel(ik.model());
        dummyJointVel.zero();
        
        kinDynOpt.setRobotState(baseOpt,sOpt,dummyVel,dummyJointVel,dummyGrav);
        double tolConstraints = 1e-7;
        double tolTargets     = 1e-6;

        // Check if the base link constraint is still valid 
        ASSERT_EQUAL_TRANSFORM_TOL(kinDynOpt.getWorldTransform("link1"), kinDynDes.getWorldTransform("link1"), tolConstraints);
        
        // Check if the target is realized 
        ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getWorldTransform(targetFrame).getPosition(), kinDynOpt.getWorldTransform(targetFrame).getPosition(), tolTargets);
    }

}

// Check the consistency of a simple humanoid wholebody IK test case
void simpleHumanoidWholeBodyIKConsistency(const iDynTree::InverseKinematicsRotationParametrization rotationParametrization)
{
    iDynTree::InverseKinematics ik;

    bool ok = ik.loadModelFromFile(getAbsModelPath("iCubGenova02.urdf"));
    ASSERT_IS_TRUE(ok);
    
    //ik.setFloatingBaseOnFrameNamed("l_foot");

    ik.setRotationParametrization(rotationParametrization);

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
    ok = ik.addFrameConstraint("l_foot",kinDynDes.getWorldTransform("l_foot"));
    ASSERT_IS_TRUE(ok);
    
    std::cerr << "kinDynDes.getWorldTransform(l_foot) : " << kinDynDes.getWorldTransform("l_foot").toString() << std::endl;

    // The relative position of the r_sole should be the initial one
    ok = ik.addFrameConstraint("r_sole",kinDynDes.getWorldTransform("r_sole"));
    ASSERT_IS_TRUE(ok);

    // The two cartesian targets should be reasonable values
    ik.setTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    ok = ik.addPositionTarget("l_elbow_1",kinDynDes.getWorldTransform("l_elbow_1"));
    ASSERT_IS_TRUE(ok);

    //ok = ik.addPositionTarget("r_elbow_1",kinDynDes.getRelativeTransform("l_sole","r_elbow_1").getPosition());
    //ASSERT_IS_TRUE(ok);

    iDynTree::Transform initialH = kinDynDes.getWorldBaseTransform();
       
    ik.setInitialCondition(&initialH,&s);
    //ik.setDesiredJointConfiguration(s);
    
    // Solve the optimization problem
    double tic = clockInSec();
    ok = ik.solve();
    double toc = clockInSec();
    std::cerr << "Inverse Kinematics solved in " << toc-tic << " seconds. " << std::endl;
    ASSERT_IS_TRUE(ok);

    iDynTree::Transform basePosOptimized;
    iDynTree::JointPosDoubleArray sOptimized(ik.model());
    sOptimized.zero();
    
    ik.getSolution(basePosOptimized,sOptimized);

    // We create a new KinDyn object to perform forward kinematics for the optimized values
    iDynTree::KinDynComputations kinDynOpt;
    kinDynOpt.loadRobotModel(ik.model());
    iDynTree::Twist dummyVel;
    dummyVel.zero();
    iDynTree::Vector3 dummyGrav;
    dummyGrav.zero();
    iDynTree::JointDOFsDoubleArray dummyJointVel(ik.model());
    dummyJointVel.zero();
    kinDynOpt.setRobotState(basePosOptimized, sOptimized, dummyVel, dummyJointVel, dummyGrav);

    // Check that the contraint and the targets are respected
    double tolConstraints = 1e-7;
    double tolTargets     = 1e-6;
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("l_foot"),kinDynOpt.getWorldTransform("l_foot"),tolConstraints);
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("r_sole"),kinDynOpt.getWorldTransform("r_sole"),tolConstraints);
    ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getWorldTransform("l_elbow_1").getPosition(),
                            kinDynOpt.getWorldTransform("l_elbow_1").getPosition(),tolTargets);

    return;
}

int main()
{
    simpleChainIK(2,13,iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    // This is not working at the moment, there is some problem with quaternion constraints
    //simpleChainIK(10,iDynTree::InverseKinematicsRotationParametrizationQuaternion);
    
    simpleHumanoidWholeBodyIKConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
    
    return EXIT_SUCCESS;
}
