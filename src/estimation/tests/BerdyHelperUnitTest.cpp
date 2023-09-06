// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/BerdyHelper.h>
#include <iDynTree/ExtWrenchesAndJointTorquesEstimator.h>

#include <iDynTree/PredictSensorsMeasurements.h>

#include "testModels.h"
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenSparseHelpers.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/SparseMatrix.h>
#include <iDynTree/ModelTestUtils.h>

#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/Dynamics.h>

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

    // Force the base linear velocity to be zero for ensure consistency with the compute buffers
    vel.baseVel().setLinearVec3(LinVelocity(0.0, 0.0, 0.0));

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

    // Propagate kinematics also inside berdy

    // Correct for the unconsistency between the input net wrenches and the residual of the RNEA
    extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex()) = extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())+genTrqs.baseWrench();

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

    // Get the angular v
    LinkIndex baseIdx = berdy.dynamicTraversal().getBaseLink()->getIndex();
    berdy.updateKinematicsFromFloatingBase(pos.jointPos(),vel.jointVel(),baseIdx,linkVels(baseIdx).getAngularVec3());

    berdy.serializeDynamicVariables(linkProperAccs,
                                    linkNetWrenchesWithoutGravity,
                                    extWrenches,
                                    intWrenches,
                                    genTrqs.jointTorques(),
                                    generalizedProperAccs.jointAcc(),
                                    d);


    // Check D and bD , in particular that D*d + bD = 0
    // Generated the Y e bY matrix and vector from berdy
    SparseMatrix<iDynTree::ColumnMajor> D, Y;
    VectorDynSize bD, bY;
    berdy.resizeAndZeroBerdyMatrices(D,bD,Y,bY);
    bool ok = berdy.getBerdyMatrices(D,bD,Y,bY);
    ASSERT_IS_TRUE(ok);

    VectorDynSize dynamicsResidual(berdy.getNrOfDynamicEquations()), zeroRes(berdy.getNrOfDynamicEquations());

    toEigen(dynamicsResidual) = toEigen(D)*toEigen(d) + toEigen(bD);

    /*
    std::cerr << "D : " << std::endl;
    std::cerr << D.description(true) << std::endl;
    std::cerr << "d :\n" << d.toString() << std::endl;
    std::cerr << "D*d :\n" << toEigen(D)*toEigen(d) << std::endl;
    std::cerr << "bD :\n" << bD.toString() << std::endl;
    */

    ASSERT_EQUAL_VECTOR(dynamicsResidual, zeroRes);
    
    if( berdy.getNrOfSensorsMeasurements() > 0 )
    {
        std::cout << "BerdyHelperUnitTest, testing sensors matrix for model " << filename <<  std::endl;

        // Generate the y vector of sensor measurements using the predictSensorMeasurements function
        VectorDynSize y(berdy.getNrOfSensorsMeasurements());
        y.zero();
        SensorsMeasurements sensMeas(berdy.sensors());
        bool ok = predictSensorsMeasurementsFromRawBuffers(berdy.model(),berdy.sensors(),berdy.dynamicTraversal(),
                                                           linkVels,linkProperAccs,intWrenches,sensMeas);
 
	    // Handle rate of change of momentum in base
        SpatialForceVector rcm;
        rcm.zero();
        for(int linkIdx = 0; linkIdx<extWrenches.getNrOfLinks(); linkIdx++)
        {
            // compute {}^{B}H_{L}
            Transform base_H_link = linkPos(baseIdx).inverse() * linkPos(linkIdx); 
            rcm = rcm + (base_H_link * extWrenches(linkIdx));
        }

        ASSERT_IS_TRUE(ok);
        ok = berdy.serializeSensorVariables(sensMeas,extWrenches,genTrqs.jointTorques(),generalizedProperAccs.jointAcc(),intWrenches,rcm,y);
        ASSERT_IS_TRUE(ok);

        // Check that y = Y*d + bY
        VectorDynSize yFromBerdy(berdy.getNrOfSensorsMeasurements());

        ASSERT_EQUAL_DOUBLE(berdy.getNrOfSensorsMeasurements(), Y.rows());
        ASSERT_EQUAL_DOUBLE(berdy.getNrOfSensorsMeasurements(), bY.size());

        toEigen(yFromBerdy) = toEigen(Y)*toEigen(d) + toEigen(bY);

        //std::cerr << "Y : " << std::endl;
        //std::cerr << Y.description(true) << std::endl;
        //std::cerr << "d :\n" << d.toString() << std::endl;
        /*
        std::cerr << "Y*d :\n" << toEigen(Y)*toEigen(d) << std::endl;
        std::cerr << "bY :\n" << bY.toString() << std::endl;
        std::cerr << "y from model:\n" << y.toString() << std::endl;
        */

        // Check if the two vectors are equal
        ASSERT_EQUAL_VECTOR(y,yFromBerdy);
    }
}

/*
 * In the ORIGINAL_BERDY_FIXED_BASE, the serialization of the
 * dynamic variables returned by getDynamicVariablesOrdering
 * should be contiguous. Check this.
 */
void testBerdyOriginalFixedBaseDynamicEquationSerialization(BerdyHelper& berdy)
{
    std::vector<iDynTree::BerdyDynamicVariable> dynVarOrdering = berdy.getDynamicVariablesOrdering();

    // Variables containing the first index not described by dynVarOrdering
    size_t accumulator=0;
    for(size_t i=0; i < dynVarOrdering.size(); i++)
    {
        ASSERT_EQUAL_DOUBLE(accumulator,dynVarOrdering[i].range.offset);
        accumulator += dynVarOrdering[i].range.size;
    }

    // Once we finish, accumulator should be equal to the number of dyn equations
    ASSERT_EQUAL_DOUBLE(berdy.getNrOfDynamicVariables(),accumulator);
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
    extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex()) = extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())+genTrqs.baseWrench();

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

    // Get the angular v
    berdy.updateKinematicsFromFixedBase(pos.jointPos(),vel.jointVel(),berdy.dynamicTraversal().getBaseLink()->getIndex(),grav);

    berdy.serializeDynamicVariables(linkProperAccs,
                                    linkNetWrenchesWithoutGravity,
                                    extWrenches,
                                    intWrenches,
                                    genTrqs.jointTorques(),
                                    generalizedProperAccs.jointAcc(),
                                    d);

    // Check D and bD , in particular that D*d + bD = 0
    // Generated the Y e bY matrix and vector from berdy
    SparseMatrix<iDynTree::ColumnMajor> D, Y;
    VectorDynSize bD, bY;
    berdy.resizeAndZeroBerdyMatrices(D,bD,Y,bY);
    bool ok = berdy.getBerdyMatrices(D,bD,Y,bY);
    ASSERT_IS_TRUE(ok);

    VectorDynSize dynamicsResidual(berdy.getNrOfDynamicEquations()), zeroRes(berdy.getNrOfDynamicEquations());

    toEigen(dynamicsResidual) = toEigen(D)*toEigen(d) + toEigen(bD);

    /*
    std::cerr << "D : " << std::endl;
    std::cerr << D.description(true) << std::endl;
    std::cerr << "d :\n" << d.toString() << std::endl;
    std::cerr << "D*d :\n" << toEigen(D)*toEigen(d) << std::endl;
    std::cerr << "bD :\n" << bD.toString() << std::endl;
    */

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
	    // Rate of Change of Momentum (RCM) is not used with this variant
        SpatialForceVector dummyRcm;
        dummyRcm.zero();
        ok = berdy.serializeSensorVariables(sensMeas,extWrenches,genTrqs.jointTorques(),generalizedProperAccs.jointAcc(),intWrenches,dummyRcm,y);
        ASSERT_IS_TRUE(ok);

        // Check that y = Y*d + bY
        VectorDynSize yFromBerdy(berdy.getNrOfSensorsMeasurements());

        toEigen(yFromBerdy) = toEigen(Y)*toEigen(d) + toEigen(bY);

        /*
        std::cerr << "Y : " << std::endl;
        std::cerr << Y.description(true) << std::endl;
        std::cerr << "d :\n" << d.toString() << std::endl;
        std::cerr << "Y*d :\n" << toEigen(Y)*toEigen(d) << std::endl;
        std::cerr << "bY :\n" << bY.toString() << std::endl;


        std::cerr << Y.description(true) << std::endl;
        std::cerr << "Testing " << berdy.getOptions().jointOnWhichTheInternalWrenchIsMeasured[0] << std::endl;
        std::cerr << intWrenches(berdy.model().getJointIndex(berdy.getOptions().jointOnWhichTheInternalWrenchIsMeasured[0])).toString() << std::endl;
        */

        // Check if the two vectors are equal
        ASSERT_EQUAL_VECTOR(y,yFromBerdy);
    }

    testBerdyOriginalFixedBaseDynamicEquationSerialization(berdy);
}

void testBerdyHelpers(std::string fileName)
{
    // \todo TODO simplify model loading (now we rely on teh ExtWrenchesAndJointTorquesEstimator
    ExtWrenchesAndJointTorquesEstimator estimator;
    bool ok = estimator.loadModelAndSensorsFromFile(fileName);

    ASSERT_IS_TRUE(estimator.sensors().isConsistent(estimator.model()));
    ASSERT_IS_TRUE(ok);

    BerdyHelper berdyHelper;

    // First test the original BERDY
    BerdyOptions options;
    options.berdyVariant = iDynTree::ORIGINAL_BERDY_FIXED_BASE;
    options.includeAllJointAccelerationsAsSensors = false;
    options.includeAllNetExternalWrenchesAsSensors = false;

    // Add one arbitary joint wrench sensor
    if( estimator.model().getNrOfJoints() > 0 )
    {
        JointIndex jntIdx = estimator.model().getNrOfJoints()/2;
        options.jointOnWhichTheInternalWrenchIsMeasured.push_back(estimator.model().getJointName(jntIdx));
    }

    ok = false;
    //ok = berdyHelper.init(estimator.model(),estimator.sensors(),options);

    if( ok )
    {
        std::cerr << "Testing ORIGINAL_BERDY_FIXED_BASE tests for model " << fileName << " because the assumptions of ORIGINAL_BERDY_FIXED_BASE are respected" << std::endl;
        //testBerdyOriginalFixedBase(berdyHelper,fileName);
        // Change the options a bit and test again
        options.includeAllNetExternalWrenchesAsDynamicVariables = false;
        berdyHelper.init(estimator.model(),estimator.sensors(),options);
        //testBerdyOriginalFixedBase(berdyHelper,fileName);
    }
    else
    {
        std::cerr << "Skipping ORIGINAL_BERDY_FIXED_BASE tests for model " << fileName << " because some assumptions of ORIGINAL_BERDY_FIXED_BASE are not respected" << std::endl;
    }

    // We test the floating base BERDY
    options.berdyVariant = iDynTree::BERDY_FLOATING_BASE;
    // For now floating berdy needs all the ext wrenches as dynamic variables
    options.includeAllNetExternalWrenchesAsDynamicVariables = true;
    ok = berdyHelper.init(estimator.model(), estimator.sensors(), options);
    ASSERT_IS_TRUE(ok);
    testBerdySensorMatrices(berdyHelper, fileName);
    
    // Test includeAllJointTorqueAsSensors option 
    options.berdyVariant = iDynTree::BERDY_FLOATING_BASE;
    // For now floating berdy needs all the ext wrenches as dynamic variables
    options.includeAllNetExternalWrenchesAsDynamicVariables = true;
    options.includeAllJointTorquesAsSensors = true;
    ok = berdyHelper.init(estimator.model(), estimator.sensors(), options);
    ASSERT_IS_TRUE(ok);
    testBerdySensorMatrices(berdyHelper, fileName);

    // We test the floating base BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES
    options.berdyVariant = iDynTree::BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES;
    options.includeAllJointTorquesAsSensors = false;
    options.includeAllJointAccelerationsAsSensors = false;
    options.includeAllNetExternalWrenchesAsSensors = true;
    options.includeAllNetExternalWrenchesAsDynamicVariables = true;
    ok = berdyHelper.init(estimator.model(), estimator.sensors(), options);
    ASSERT_IS_TRUE(ok);
    testBerdySensorMatrices(berdyHelper, fileName);

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
