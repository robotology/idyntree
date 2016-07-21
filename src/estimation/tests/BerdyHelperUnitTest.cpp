/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Estimation/BerdyHelper.h>
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include <iDynTree/Sensors/PredictSensorsMeasurements.h>

#include "testModels.h"
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void testBerdySensorMatrices(BerdyHelper & berdy, std::string filename)
{
    // Check the concistency of the sensor matrices
    // Generate a random pos, vel, acc, external wrenches
    FreeFloatingPos pos(berdy.model());
    FreeFloatingVel vel(berdy.model());
    FreeFloatingAcc generalizedProperAccs(berdy.model());
    LinkNetExternalWrenches extWrenches(berdy.model());

    getRandomInverseDynamicsInputs(pos,vel,generalizedProperAccs,extWrenches);

    LinkPositions linkPos(berdy.model());
    LinkVelArray  linkVels(berdy.model());
    LinkAccArray  linkProperAccs(berdy.model());

    LinkInternalWrenches intWrenches(berdy.model());
    FreeFloatingGeneralizedTorques genTrqs(berdy.model());

    // Compute consistent joint torques and internal forces using inverse dynamics
    ForwardPosVelAccKinematics(berdy.model(),berdy.dynamicTraversal(),
                               pos, vel, generalizedProperAccs,
                               linkPos,linkVels,linkProperAccs);
    RNEADynamicPhase(berdy.model(),berdy.dynamicTraversal(),
                     pos.jointPos(),linkVels,linkProperAccs,
                     extWrenches,intWrenches,genTrqs);

    // Correct for the unconsistency between the input net wrenches and the residual of the RNEA
    extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex()) = extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())-genTrqs.baseWrench();

    // Generate the d vector of dynamical variables
    VectorDynSize d(berdy.getNrOfDynamicVariables());

    // LinkNewInternalWrenches (necessary for the old-style berdy)
    LinkNetTotalWrenchesWithoutGravity linkNetWrenchesWithoutGravity(berdy.model());

    for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < berdy.model().getNrOfLinks(); visitedLinkIndex++)
     {
         LinkConstPtr visitedLink = berdy.model().getLink(visitedLinkIndex);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAccs(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVels(visitedLinkIndex);
         linkNetWrenchesWithoutGravity(visitedLinkIndex) = I*properAcc + v*(I*v);
     }

    berdy.serializeDynamicVariables(linkProperAccs,
                                    linkNetWrenchesWithoutGravity,
                                    extWrenches,
                                    intWrenches,
                                    genTrqs.jointTorques(),
                                    generalizedProperAccs.jointAcc(),
                                    d);

    // Get the angular v
    LinkIndex baseIdx = berdy.dynamicTraversal().getBaseLink()->getIndex();
    berdy.updateKinematicsFromFloatingBase(pos.jointPos(),vel.jointVel(),baseIdx,linkVels(baseIdx).getAngularVec3());

    // Check D and bD , in particular that D*d + bD = 0
    // Generated the Y e bY matrix and vector from berdy
    MatrixDynSize D, Y;
    VectorDynSize bD, bY;
    berdy.resizeBerdyMatrices(D,bD,Y,bY);
    bool ok = berdy.getBerdyMatrices(D,bD,Y,bY);
    ASSERT_IS_TRUE(ok);

    VectorDynSize dynamicsResidual(berdy.getNrOfDynamicEquations()), zeroRes(berdy.getNrOfDynamicEquations());

    toEigen(dynamicsResidual) = toEigen(D)*toEigen(d) + toEigen(bD);

    if( berdy.getNrOfSensorsMeasurements() > 0 )
    {
        std::cout << "BerdyHelperUnitTest, testing sensors matrix for model " << filename <<  std::endl;

        // Generate the y vector of sensor measurements using the predictSensorMeasurements function
        VectorDynSize y(berdy.getNrOfSensorsMeasurements());
        y.zero();
        SensorsMeasurements sensMeas(berdy.sensors());
        bool ok = predictSensorsMeasurementsFromRawBuffers(berdy.model(),berdy.sensors(),berdy.dynamicTraversal(),
                                                           linkVels,linkProperAccs,intWrenches,sensMeas);

        ASSERT_IS_TRUE(ok);
        ok = sensMeas.toVector(y);
        ASSERT_IS_TRUE(ok);

        // Check that y = Y*d + bY
        VectorDynSize yFromBerdy(berdy.getNrOfSensorsMeasurements());

        toEigen(yFromBerdy) = toEigen(Y)*toEigen(d) + toEigen(bY);

        // Check if the two vectors are equal
        ASSERT_EQUAL_VECTOR(y,yFromBerdy);
    }
}

void testBerdyOriginalFixedBase(BerdyHelper & berdy, std::string filename)
{
    // Check the concistency of the sensor matrices
    // Generate a random pos, vel, acc, external wrenches
    FreeFloatingPos pos(berdy.model());
    FreeFloatingVel vel(berdy.model());
    FreeFloatingAcc generalizedProperAccs(berdy.model());
    LinkNetExternalWrenches extWrenches(berdy.model());

    getRandomInverseDynamicsInputs(pos,vel,generalizedProperAccs,extWrenches);

    Vector3 grav;
    grav.zero();
    grav(2) = -10;

    Vector3 baseProperAcc;
    baseProperAcc.zero();
    baseProperAcc(2) = -grav(2);

    // Set the base variables to zero for the fixed base case
    pos.worldBasePos() = Transform::Identity();
    vel.baseVel().zero();
    generalizedProperAccs.baseAcc().zero();
    generalizedProperAccs.baseAcc().setLinearVec3(baseProperAcc);

    LinkPositions linkPos(berdy.model());
    LinkVelArray  linkVels(berdy.model());
    LinkAccArray  linkProperAccs(berdy.model());

    LinkInternalWrenches intWrenches(berdy.model());
    FreeFloatingGeneralizedTorques genTrqs(berdy.model());

    // Compute consistent joint torques and internal forces using inverse dynamics
    ForwardPosVelAccKinematics(berdy.model(),berdy.dynamicTraversal(),
                               pos, vel, generalizedProperAccs,
                               linkPos,linkVels,linkProperAccs);
    RNEADynamicPhase(berdy.model(),berdy.dynamicTraversal(),
                     pos.jointPos(),linkVels,linkProperAccs,
                     extWrenches,intWrenches,genTrqs);

    // Correct for the unconsistency between the input net wrenches and the residual of the RNEA
    extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex()) = extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())-genTrqs.baseWrench();

    // Generate the d vector of dynamical variables
    VectorDynSize d(berdy.getNrOfDynamicVariables());

    // LinkNewInternalWrenches (necessary for the old-style berdy)
    LinkNetTotalWrenchesWithoutGravity linkNetWrenchesWithoutGravity(berdy.model());

    for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < berdy.model().getNrOfLinks(); visitedLinkIndex++)
     {
         LinkConstPtr visitedLink = berdy.model().getLink(visitedLinkIndex);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAccs(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVels(visitedLinkIndex);
         linkNetWrenchesWithoutGravity(visitedLinkIndex) = I*properAcc + v*(I*v);
     }

    berdy.serializeDynamicVariables(linkProperAccs,
                                    linkNetWrenchesWithoutGravity,
                                    extWrenches,
                                    intWrenches,
                                    genTrqs.jointTorques(),
                                    generalizedProperAccs.jointAcc(),
                                    d);

    // Get the angular v
    berdy.updateKinematicsFromFixedBase(pos.jointPos(),vel.jointVel(),berdy.dynamicTraversal().getBaseLink()->getIndex(),grav);

    // Check D and bD , in particular that D*d + bD = 0
    // Generated the Y e bY matrix and vector from berdy
    MatrixDynSize D, Y;
    VectorDynSize bD, bY;
    berdy.resizeBerdyMatrices(D,bD,Y,bY);
    bool ok = berdy.getBerdyMatrices(D,bD,Y,bY);
    ASSERT_IS_TRUE(ok);

    VectorDynSize dynamicsResidual(berdy.getNrOfDynamicEquations()), zeroRes(berdy.getNrOfDynamicEquations());

    toEigen(dynamicsResidual) = toEigen(D)*toEigen(d) + toEigen(bD);

    std::cerr << "D : " << std::endl;
    std::cerr << D.toString() << std::endl;
    std::cerr << "d :\n" << d.toString() << std::endl;
    std::cerr << "D*d :\n" << toEigen(D)*toEigen(d) << std::endl;
    std::cerr << "bD :\n" << bD.toString() << std::endl;

    ASSERT_EQUAL_VECTOR(dynamicsResidual,zeroRes);


    if( berdy.getNrOfSensorsMeasurements() > 0 )
    {
        std::cout << "BerdyHelperUnitTest, testing sensors matrix for model " << filename <<  std::endl;

        // Generate the y vector of sensor measurements using the predictSensorMeasurements function
        VectorDynSize y(berdy.getNrOfSensorsMeasurements());
        y.zero();
        SensorsMeasurements sensMeas(berdy.sensors());
        ok = predictSensorsMeasurementsFromRawBuffers(berdy.model(),berdy.sensors(),berdy.dynamicTraversal(),
                                                           linkVels,linkProperAccs,intWrenches,sensMeas);

        ASSERT_IS_TRUE(ok);
        ok = sensMeas.toVector(y);
        ASSERT_IS_TRUE(ok);

        // Check that y = Y*d + bY
        VectorDynSize yFromBerdy(berdy.getNrOfSensorsMeasurements());

        toEigen(yFromBerdy) = toEigen(Y)*toEigen(d) + toEigen(bY);

        // Check if the two vectors are equal
        ASSERT_EQUAL_VECTOR(y,yFromBerdy);
    }
}

void testBerdyHelpers(std::string fileName)
{
    // \todo TODO simplify model loading (now we rely on teh ExtWrenchesAndJointTorquesEstimator
    ExtWrenchesAndJointTorquesEstimator estimator;
    bool ok = estimator.loadModelAndSensorsFromFile(fileName);

    ASSERT_IS_TRUE(ok);

    BerdyHelper berdyHelper;

    // First test the original BERDY
    ok = berdyHelper.init(estimator.model(),estimator.sensors(),iDynTree::ORIGINAL_BERDY_FIXED_BASE);

    if( ok )
    {
        std::cerr << "Testing ORIGINAL_BERDY_FIXED_BASE tests for model " << fileName << " because the assumptions of ORIGINAL_BERDY_FIXED_BASE are respected" << std::endl;
        testBerdyOriginalFixedBase(berdyHelper,fileName);
    }
    else
    {
        std::cerr << "Skipping ORIGINAL_BERDY_FIXED_BASE tests for model " << fileName << " because some assumptions of ORIGINAL_BERDY_FIXED_BASE are not respected" << std::endl;
    }

    // We test the floating base BERDY (still needs to be added)
    //ok = berdyHelper.init(estimator.model(),estimator.sensors(),iDynTree::BERDY_FLOATING_BASE);
    //ASSERT_IS_TRUE(ok);
    //testBerdySensorMatrices(berdyHelper,fileName);

}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "BerdyHelperUnitTest, testing file " << std::string(IDYNTREE_TESTS_URDFS[mdl]) <<  std::endl;
        testBerdyHelpers(urdfFileName);
    }

    return EXIT_SUCCESS;
}
