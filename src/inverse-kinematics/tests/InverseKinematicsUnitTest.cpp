/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/InverseKinematics.h>
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/ModelIO/ModelLoader.h>

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


inline iDynTree::JointPosDoubleArray getRandomJointPositionsCloseTo(const iDynTree::Model& model, iDynTree::VectorDynSize s, double maxDelta)
{
    iDynTree::JointPosDoubleArray vec(model);

    assert(vec.size() == model.getNrOfPosCoords());
    for(iDynTree::JointIndex jntIdx=0; jntIdx < model.getNrOfJoints(); jntIdx++)
    {
        iDynTree::IJointConstPtr jntPtr = model.getJoint(jntIdx);
        if( jntPtr->hasPosLimits() )
        {
            for(int i=0; i < jntPtr->getNrOfPosCoords(); i++)
            {
                int dofIndex = jntPtr->getDOFsOffset()+i;
                double max = std::min(jntPtr->getMaxPosLimit(i),s(dofIndex)+maxDelta);
                double min = std::max(jntPtr->getMinPosLimit(i),s(dofIndex)-maxDelta);
                vec(dofIndex) = iDynTree::getRandomDouble(min,max);
            }
        }
        else
        {
            for(int i=0; i < jntPtr->getNrOfPosCoords(); i++)
            {
                int dofIndex = jntPtr->getDOFsOffset()+i;
                vec(jntPtr->getDOFsOffset()+i) = iDynTree::getRandomDouble(s(dofIndex)-maxDelta,s(dofIndex)+maxDelta);
            }
        }
    }

    return vec;
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
        iDynTree::Model chain = iDynTree::getRandomChain(i, 10, noFixedJoints);

        ASSERT_EQUAL_DOUBLE(i, chain.getNrOfDOFs());

        // Name of the targetFrame the leaf added by getRandomChain
        std::string targetFrame = "link" + iDynTree::int2string(i - 1);

        // Create IK
        iDynTree::InverseKinematics ik;
        bool ok = ik.setModel(chain);
        ASSERT_IS_TRUE(ok);
        // Always express the target as cost
        ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);

        // Use the requested parametrization
        ik.setRotationParametrization(rotationParametrization);

        ik.setCostTolerance(1e-6);
        ik.setConstraintsTolerance(1e-7);

        // Create also a KinDyn object to perform forward kinematics for the desired values and the optimized ones
        iDynTree::KinDynComputations kinDynDes;
        ok = kinDynDes.loadRobotModel(ik.fullModel());
        ASSERT_IS_TRUE(ok);
        iDynTree::KinDynComputations kinDynOpt;
        ok = kinDynOpt.loadRobotModel(ik.fullModel());
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
        // Set a max delta to avoid local minima in testing
        double maxDelta = 0.01;
        iDynTree::JointPosDoubleArray sInitial = getRandomJointPositionsCloseTo(ik.fullModel(), s, maxDelta);
        ik.setFullJointsInitialCondition(&(baseDes), &(sInitial));

        // Add a random desired position
        iDynTree::JointPosDoubleArray sDesired = getRandomJointPositions(ik.fullModel());
        ik.setDesiredFullJointsConfiguration(sDesired, 1e-12);

        // Start from the initial one
        double tic = clockInSec();
        ok = ik.solve();
        double toc = clockInSec();

        ASSERT_IS_TRUE(ok);

        // Get the solution
        iDynTree::Transform baseOpt = iDynTree::Transform::Identity();
        iDynTree::JointPosDoubleArray sOpt(ik.fullModel());
        ik.getFullJointsSolution(baseOpt, sOpt);
        iDynTree::Twist dummyVel;
        dummyVel.zero();
        iDynTree::Vector3 dummyGrav;
        dummyGrav.zero();
        iDynTree::JointDOFsDoubleArray dummyJointVel(ik.fullModel());
        dummyJointVel.zero();

        kinDynOpt.setRobotState(baseOpt, sOpt, dummyVel, dummyJointVel, dummyGrav);
        double tolConstraints = 1e-6;
        double tolTargets     = 1e-3;

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
    ok = kinDynDes.loadRobotModel(ik.fullModel());
    ASSERT_IS_TRUE(ok);

    // Create a random vector of internal joint positions
    // used to get reasonable values for the constraints and the targets
    iDynTree::JointPosDoubleArray s = getRandomJointPositions(kinDynDes.model());
    ok = kinDynDes.setJointPos(s);
    ASSERT_IS_TRUE(ok);

    // Create a simple IK problem with the foot constraint

    // The l_sole frame is our world absolute frame (i.e. {}^A H_{l_sole} = identity
    ok = ik.addFrameConstraint("l_foot", kinDynDes.getWorldTransform("l_foot"));
    ASSERT_IS_TRUE(ok);

    std::cerr << "kinDynDes.getWorldTransform(l_foot) : " << kinDynDes.getWorldTransform("l_foot").toString() << std::endl;

    // The relative position of the r_sole should be the initial one
    ok = ik.addFrameConstraint("r_sole", kinDynDes.getWorldTransform("r_sole"));
    ASSERT_IS_TRUE(ok);

    // The two cartesian targets should be reasonable values
    ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    ok = ik.addPositionTarget("l_elbow_1", kinDynDes.getWorldTransform("l_elbow_1"));
    ASSERT_IS_TRUE(ok);

    //ok = ik.addPositionTarget("r_elbow_1",kinDynDes.getRelativeTransform("l_sole","r_elbow_1").getPosition());
    //ASSERT_IS_TRUE(ok);

    iDynTree::Transform initialH = kinDynDes.getWorldBaseTransform();

    ik.setFullJointsInitialCondition(&initialH, &s);
    ik.setDesiredFullJointsConfiguration(s, 1e-15);

    // Solve the optimization problem
    double tic = clockInSec();
    ok = ik.solve();
    double toc = clockInSec();
    std::cerr << "Inverse Kinematics solved in " << toc-tic << " seconds. " << std::endl;
    ASSERT_IS_TRUE(ok);

    iDynTree::Transform basePosOptimized;
    iDynTree::JointPosDoubleArray sOptimized(ik.fullModel());
    sOptimized.zero();

    ik.getFullJointsSolution(basePosOptimized, sOptimized);

    // We create a new KinDyn object to perform forward kinematics for the optimized values
    iDynTree::KinDynComputations kinDynOpt;
    kinDynOpt.loadRobotModel(ik.fullModel());
    iDynTree::Twist dummyVel;
    dummyVel.zero();
    iDynTree::Vector3 dummyGrav;
    dummyGrav.zero();
    iDynTree::JointDOFsDoubleArray dummyJointVel(ik.fullModel());
    dummyJointVel.zero();
    kinDynOpt.setRobotState(basePosOptimized, sOptimized, dummyVel, dummyJointVel, dummyGrav);

    // Check that the contraint and the targets are respected
    double tolConstraints = 1e-7;
    double tolTargets     = 1e-6;
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("l_foot"), kinDynOpt.getWorldTransform("l_foot"), tolConstraints);
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("r_sole"), kinDynOpt.getWorldTransform("r_sole"), tolConstraints);
    ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getWorldTransform("l_elbow_1").getPosition(),
                            kinDynOpt.getWorldTransform("l_elbow_1").getPosition(), tolTargets);

    return;
}

// Check the consistency of a simple humanoid wholebody IK test case, setting a CoM target.
void simpleHumanoidWholeBodyIKCoMConsistency(const iDynTree::InverseKinematicsRotationParametrization rotationParametrization,
                                             const iDynTree::InverseKinematicsTreatTargetAsConstraint targetResolutionMode,
                                             size_t dofsToBeRemoved = 0)
{
    iDynTree::InverseKinematics ik;

    ik.setVerbosity(3);

    std::vector<std::string> consideredJoints;
    iDynTree::ModelLoader loader;
    loader.loadModelFromFile(getAbsModelPath("iCubGenova02.urdf"));
    if (dofsToBeRemoved > 0) {
        consideredJoints.reserve(loader.model().getNrOfDOFs());
        // TODO: 1 to 1 joints to dofs
        // Also.. I assume 0 dofs joints are at the end.
        //fill all the joints
        for (int index = 0; index < loader.model().getNrOfDOFs(); ++index) {
            consideredJoints.push_back(loader.model().getJointName(index));
        }
    }
    std::vector<int> toBeRemoved;
    toBeRemoved.reserve(dofsToBeRemoved);
    for (size_t index = 0; index < dofsToBeRemoved; ++index) {
        int indexToBeRemoved = -1;
        do {
            indexToBeRemoved = rand() % loader.model().getNrOfDOFs();
        } while (std::find(toBeRemoved.begin(), toBeRemoved.end(), indexToBeRemoved) != toBeRemoved.end());
        toBeRemoved.push_back(indexToBeRemoved);
    }
    std::sort(toBeRemoved.begin(), toBeRemoved.end());

    for (std::vector<int>::const_reverse_iterator rit = toBeRemoved.rbegin(); rit != toBeRemoved.rend(); ++rit) {
        std::cerr << "Removing DOF " << *(consideredJoints.begin() + (*rit)) << "(" << *rit << ")" <<std::endl;
        consideredJoints.erase(consideredJoints.begin() + (*rit));
    }

    bool ok = ik.setModel(loader.model(), consideredJoints);
    ASSERT_IS_TRUE(ok);

    //ik.setFloatingBaseOnFrameNamed("l_foot");

    ik.setRotationParametrization(rotationParametrization);
    ik.setDefaultTargetResolutionMode(targetResolutionMode);
    
    // Create also a KinDyn object to perform forward kinematics for the desired values
    iDynTree::KinDynComputations kinDynDes;
    ok = kinDynDes.loadRobotModel(ik.fullModel());
    ASSERT_IS_TRUE(ok);

    // Create a random vector of internal joint positions
    // used to get reasonable values for the constraints and the targets
    iDynTree::JointPosDoubleArray s = getRandomJointPositions(kinDynDes.model());
    ok = kinDynDes.setJointPos(s);
    ASSERT_IS_TRUE(ok);
    iDynTree::Transform initialH = kinDynDes.getWorldBaseTransform();

    // set robot configuration
    ik.setCurrentRobotConfiguration(initialH, s);

    // Create a simple IK problem with the foot constraint

    // The l_sole frame is our world absolute frame (i.e. {}^A H_{l_sole} = identity
    ok = ik.addFrameConstraint("l_foot",kinDynDes.getWorldTransform("l_foot"));
    ASSERT_IS_TRUE(ok);

    std::cerr << "kinDynDes.getWorldTransform(l_foot) : " << kinDynDes.getWorldTransform("l_foot").toString() << std::endl;

    // The relative position of the r_sole should be the initial one
    ok = ik.addFrameConstraint("r_sole", kinDynDes.getWorldTransform("r_sole"));
    ASSERT_IS_TRUE(ok);

    // The two cartesian targets should be reasonable values
    iDynTree::Position comDes = kinDynDes.getCenterOfMassPosition();
    ik.setCOMTarget(comDes, 10);
    ik.setCOMAsConstraintTolerance(1e-8);
    
    //ok = ik.addPositionTarget("r_elbow_1",kinDynDes.getRelativeTransform("l_sole","r_elbow_1").getPosition());
    //ASSERT_IS_TRUE(ok);

    ik.setFullJointsInitialCondition(&initialH, &s);
    ik.setDesiredFullJointsConfiguration(s, 1e-15);

    // Solve the optimization problem
    double tic = clockInSec();
    ok = ik.solve();
    double toc = clockInSec();
    std::cerr << "Inverse Kinematics solved in " << toc-tic << " seconds. " << std::endl;
    ASSERT_IS_TRUE(ok);

    iDynTree::Transform basePosOptimized;
    iDynTree::VectorDynSize sOptimized(ik.fullModel().getNrOfDOFs()), sOptimizedFromReduced(ik.fullModel().getNrOfDOFs());
    sOptimized.zero();
    sOptimizedFromReduced.zero();
    iDynTree::VectorDynSize sReducedOptimized(ik.reducedModel().getNrOfDOFs());
    sReducedOptimized.zero();

    ik.getFullJointsSolution(basePosOptimized, sOptimized);
    ik.getReducedSolution(basePosOptimized, sReducedOptimized);
    // now map sReduced to sOptimizedFromReduced
    

    // We create a new KinDyn object to perform forward kinematics for the optimized values
    iDynTree::KinDynComputations kinDynOpt;
    kinDynOpt.loadRobotModel(ik.fullModel());
    iDynTree::Twist dummyVel;
    dummyVel.zero();
    iDynTree::Vector3 dummyGrav;
    dummyGrav.zero();
    iDynTree::JointDOFsDoubleArray dummyJointVel(ik.fullModel());
    dummyJointVel.zero();

    kinDynOpt.setRobotState(basePosOptimized, sOptimized, dummyVel, dummyJointVel, dummyGrav);

    // Check that the contraint and the targets are respected
    double tolConstraints = 1e-7;
    double tolTargets     = 1e-6;
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("l_foot"), kinDynOpt.getWorldTransform("l_foot"), tolConstraints);
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("r_sole"), kinDynOpt.getWorldTransform("r_sole"), tolConstraints);
    ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getCenterOfMassPosition(),
                            kinDynOpt.getCenterOfMassPosition(), tolTargets);

    return;
}


// Check the consistency of a simple humanoid wholebody IK test case, setting a CoM target.
void simpleHumanoidWholeBodyIKCoMandChestConsistency(const iDynTree::InverseKinematicsRotationParametrization rotationParametrization,
                                             const iDynTree::InverseKinematicsTreatTargetAsConstraint targetResolutionMode)
{
    iDynTree::InverseKinematics ik;

    ik.setMaxIterations(500);
    ik.setVerbosity(3);
    
    bool ok = ik.loadModelFromFile(getAbsModelPath("iCubGenova02.urdf"));
    ASSERT_IS_TRUE(ok);

    //ik.setFloatingBaseOnFrameNamed("l_foot");

    ik.setRotationParametrization(rotationParametrization);
    ik.setDefaultTargetResolutionMode(targetResolutionMode);

    // Create also a KinDyn object to perform forward kinematics for the desired values
    iDynTree::KinDynComputations kinDynDes;
    ok = kinDynDes.loadRobotModel(ik.fullModel());
    ASSERT_IS_TRUE(ok);

    // Create a random vector of internal joint positions
    // used to get reasonable values for the constraints and the targets
    iDynTree::JointPosDoubleArray s = getRandomJointPositions(kinDynDes.model());
    ok = kinDynDes.setJointPos(s);
    ASSERT_IS_TRUE(ok);

    // Create a simple IK problem with the foot constraint

    // The l_sole frame is our world absolute frame (i.e. {}^A H_{l_sole} = identity
    ok = ik.addFrameConstraint("l_foot", kinDynDes.getWorldTransform("l_foot"));
    ASSERT_IS_TRUE(ok);

    std::cerr << "kinDynDes.getWorldTransform(l_foot) : " << kinDynDes.getWorldTransform("l_foot").toString() << std::endl;

    // The relative position of the r_sole should be the initial one
    ok = ik.addFrameConstraint("r_sole",kinDynDes.getWorldTransform("r_sole"));
    ASSERT_IS_TRUE(ok);

    // The two cartesian targets should be reasonable values
    iDynTree::Position comDes = kinDynDes.getCenterOfMassPosition();
    ik.setCOMTarget(comDes, 1);
    ik.setCOMAsConstraintTolerance(1e-8);


    ok = ik.addRotationTarget("chest", kinDynDes.getWorldTransform("chest").getRotation(), 100);
    ASSERT_IS_TRUE(ok);
    ik.setTargetResolutionMode("chest", iDynTree::InverseKinematicsTreatTargetAsConstraintNone);

    iDynTree::Transform initialH = kinDynDes.getWorldBaseTransform();

    ik.setFullJointsInitialCondition(&initialH, &s);
    ik.setDesiredFullJointsConfiguration(s, 1e-15);

    // Solve the optimization problem
    double tic = clockInSec();
    ok = ik.solve();
    double toc = clockInSec();
    std::cerr << "Inverse Kinematics solved in " << toc-tic << " seconds. " << std::endl;
    ASSERT_IS_TRUE(ok);

    iDynTree::Transform basePosOptimized;
    iDynTree::JointPosDoubleArray sOptimized(ik.fullModel());
    sOptimized.zero();

    ik.getFullJointsSolution(basePosOptimized, sOptimized);

    // We create a new KinDyn object to perform forward kinematics for the optimized values
    iDynTree::KinDynComputations kinDynOpt;
    kinDynOpt.loadRobotModel(ik.fullModel());
    iDynTree::Twist dummyVel;
    dummyVel.zero();
    iDynTree::Vector3 dummyGrav;
    dummyGrav.zero();
    iDynTree::JointDOFsDoubleArray dummyJointVel(ik.fullModel());
    dummyJointVel.zero();
    kinDynOpt.setRobotState(basePosOptimized, sOptimized, dummyVel, dummyJointVel, dummyGrav);

    // Check that the contraint and the targets are respected
    double tolConstraints = 1e-7;
    double tolTargets     = 1e-6;
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("l_foot"), kinDynOpt.getWorldTransform("l_foot"), tolConstraints);
    ASSERT_EQUAL_TRANSFORM_TOL(kinDynDes.getWorldTransform("r_sole"), kinDynOpt.getWorldTransform("r_sole"), tolConstraints);
    ASSERT_EQUAL_VECTOR_TOL(kinDynDes.getCenterOfMassPosition(),
                            kinDynOpt.getCenterOfMassPosition(), tolTargets);
    ASSERT_EQUAL_MATRIX_TOL(kinDynDes.getWorldTransform("chest").getRotation(), kinDynOpt.getWorldTransform("chest").getRotation(), tolTargets);

    return;
}

int main()
{
    // Improve repetability (at least in the same platform)
    srand(0);

    simpleChainIK(2, 13, iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    // This is not working at the moment, there is some problem with quaternion constraints
    //simpleChainIK(10,iDynTree::InverseKinematicsRotationParametrizationQuaternion);

    simpleHumanoidWholeBodyIKConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
    simpleHumanoidWholeBodyIKCoMConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw, iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    simpleHumanoidWholeBodyIKCoMConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw, iDynTree::InverseKinematicsTreatTargetAsConstraintFull);
    simpleHumanoidWholeBodyIKCoMandChestConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw, iDynTree::InverseKinematicsTreatTargetAsConstraintFull);

    for (size_t i = 1; i < 10; i++) {
        std::cerr << "Removing " << i << " Dofs" << std::endl;
        simpleHumanoidWholeBodyIKCoMConsistency(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw, iDynTree::InverseKinematicsTreatTargetAsConstraintFull, i);
    }


    return EXIT_SUCCESS;
}
