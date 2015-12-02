/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/ModelIO/URDFModelImport.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/DynamicsLinearization.h>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMassMatrix.h>

#include "testModels.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

/**
 * Fill a single column of the linearization matrix
 *
 */
void fillLowerColumnLinearizationMatrix(FreeFloatingStateLinearization & A,
                                        FreeFloatingAcc & acc,
                                        FreeFloatingAcc & perturbedAcc,
                                        double step,
                                        size_t column)
{
    size_t nrOfDofs = acc.getNrOfDOFs();
    toEigen(A).block<6,1>(6+nrOfDofs,column) = (toEigen(perturbedAcc.baseAcc())-toEigen(acc.baseAcc()))/step;
    std::cout << (perturbedAcc.baseAcc()-acc.baseAcc()).toString() << std::endl;

    for(size_t dof = 0; dof < acc.getNrOfDOFs(); dof++ )
    {
        A(6+nrOfDofs+6+dof,column) = (perturbedAcc.jointAcc()(dof)-acc.jointAcc()(dof))/step;
    }
}

/*
 * Compute the left-trivialized linearization,
 * as described in [fill with left-trivialized repot when available],
 * using numerical derivatives (performs numerical derivatives,
 * allocate
 */
bool ForwardDynamicsLinearizationNumerical(const Model& model,
                                           const Traversal& traversal,
                                           const FreeFloatingPos& robotPos,
                                           const FreeFloatingVel& robotVel,
                                           const LinkExternalWrenches & linkExtWrenches,
                                           const JointDoubleArray & jointTorques,
                                                 ArticulatedBodyAlgorithmInternalBuffers bufs,
                                                 FreeFloatingAcc & robotAcc,
                                                 FreeFloatingStateLinearization & A)
{
    // First run the normal aba to get actual robot acceleration
    bool ok = true;

    ok = ok & ArticulatedBodyAlgorithm(model,traversal,robotPos,robotVel,
                                      linkExtWrenches,jointTorques,bufs,robotAcc);

    FreeFloatingAcc perturbedAcc(model);

    A.zero();

    // Dealing with the easy part for now: derivative of acceleration
    // with respect to the base twist, joint position and accelerations
    FreeFloatingPos perturbedRobotPos(model);
    perturbedRobotPos.worldBasePos() = robotPos.worldBasePos();
    toEigen(perturbedRobotPos.jointPos()) = toEigen(robotPos.jointPos());

    // Copy its from the input
    FreeFloatingVel perturbedRobotVel(model);
    perturbedRobotVel.baseVel() = robotVel.baseVel();
    toEigen(perturbedRobotVel.jointVel()) = toEigen(robotVel.jointVel());

    // Step of the numerical derivatives
    double step = 1e-4;

    // Don't doing derivative with respect to the base position for now

    // Joint position derivative
    for(size_t h = 0; h < model.getNrOfDOFs(); h++)
    {
    }

    // Base vel derivative
    for(size_t h = 0; h < 6; h++)
    {
        // Add perturbation to the h element of the base velocity
        perturbedRobotVel.baseVel()(h) = robotVel.baseVel()(h) + step;

        // We run the ABA with the perturbed input
        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPos,perturbedRobotVel,
                                 linkExtWrenches,jointTorques,bufs,perturbedAcc);

        // Fill the matrix column
        fillLowerColumnLinearizationMatrix(A,robotAcc,perturbedAcc,step,h+6+model.getNrOfDOFs());

        // Restored the old value
        perturbedRobotVel.baseVel()(h) = robotVel.baseVel()(h);
    }

    // Joint vel derivative
    for(size_t h = 0; h < model.getNrOfDOFs(); h++)
    {
    }

    return ok;
}


/**
 * Check that the numerical linearization of the ABA and
 * its symbolical derivation match.
 */
void checkABAandABALinearizationAreConsistent(const Model & model,
                                              const Traversal & traversal)
{
    // Input data
     // Allocate input for both algorithms : robot position, velocity
    // and link external wrenches
    LinkExternalWrenches linkExtWrenches(model);
    FreeFloatingPos   robotPos(model);
    FreeFloatingVel   robotVel(model);
    FreeFloatingAcc   robotAcc(model);

    // Input for direct dynamics algorithms
    // and output for inverse dynamics : joint torques
    JointDoubleArray jntTorques(model);

    // Fill the input to forward dynamics with random data
    robotPos.worldBasePos() = getRandomTransform();
    robotVel.baseVel() = getRandomTwist();
    getRandomVector(robotPos.jointPos());
    getRandomVector(robotVel.jointVel());
    for(unsigned int link=0; link < model.getNrOfLinks(); link++ )
    {
        linkExtWrenches(link) = getRandomWrench();
    }
    getRandomVector(jntTorques);

    // Create buffers for the algorithms
    ArticulatedBodyAlgorithmInternalBuffers abaBufs(model);
    ForwardDynamicsLinearizationInternalBuffers bufs(model);

    FreeFloatingStateLinearization A(model);
    FreeFloatingStateLinearization Anumerical(model);

    // Compute matrix with numerical derivatives
    ForwardDynamicsLinearizationNumerical(model,traversal,robotPos,robotVel,
                                          linkExtWrenches,jntTorques,abaBufs,
                                          robotAcc,Anumerical);

    ForwardDynamicsLinearization(model,traversal,robotPos,robotVel,
                                 linkExtWrenches,jntTorques,bufs,
                                 robotAcc,A);

    std::cout << " A : " << std::endl;
    std::cout << A.toString() << std::endl;

    std::cout << " A (computed with numerical derivatives): " << std::endl;
    std::cout << Anumerical.toString() << std::endl;

    ASSERT_EQUAL_MATRIX(A,Anumerical);

    return;
}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking DynamicsLinearization test on " << urdfFileName << std::endl;
        Model model;
        bool ok = modelFromURDF(urdfFileName,model);
        assert(ok);
        Traversal traversal;
        ok = model.computeFullTreeTraversal(traversal);
        assert(ok);
        checkABAandABALinearizationAreConsistent(model,traversal);
    }
}
