/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/BerdyHelper.h>
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include <iDynTree/Sensors/PredictSensorsMeasurements.h>

#include "testModels.h"
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/TransformDerivative.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <iDynTree/KinDynComputations.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

//TODO remove
/**
 * Get random robot positions, velocities and accelerations
 * and external wrenches to be given as an input to InverseDynamics.
 */
inline bool getRandomInverseDynamicsInputsCustom(FreeFloatingPos& pos,
                                                FreeFloatingVel& vel,
                                                FreeFloatingAcc& acc,
                                                LinkNetExternalWrenches& extWrenches)
{
    //srand(time(NULL));
    pos.worldBasePos() = Transform::Identity();
    //pos.worldBasePos().setPosition(getRandomPosition());
    //pos.worldBasePos().setPosition(Position(3,5,0));
    //pos.worldBasePos().setRotation(Rotation::RPY(0., M_PI/2., 0.));
    //pos.worldBasePos() = getRandomTransform();
    
    vel.baseVel().zero();
    acc.baseAcc().zero();


    //acc.baseAcc()(0) = 17;
    //for(int i=0;i<6; i++) acc.baseAcc()(i) = 17;

    acc.baseAcc() = getRandomTwist();
    //acc.baseAcc()(0) = 11.;
    //acc.baseAcc()(1) = 13.;
    //acc.baseAcc()(2) = 17.;
    //acc.baseAcc()(3) = 7.;
    //acc.baseAcc()(4) = 7.;
    //acc.baseAcc()(5) = 9.;
    
    vel.baseVel() =  getRandomTwist();
    //vel.baseVel()(0) = 2.;
    //vel.baseVel()(1) = 3.;
    //vel.baseVel()(2) = 5.;
    //vel.baseVel()(3) = 7.;

    for(unsigned int jnt=0; jnt < pos.getNrOfPosCoords(); jnt++)
    {
        pos.jointPos()(jnt) = 0;
    }

    for(unsigned int jnt=0; jnt < vel.getNrOfDOFs(); jnt++)
    {
        vel.jointVel()(jnt) = 0;
        acc.jointAcc()(jnt) = 0;
    }

    return true;
}

Vector6 computeROCMInBaseUsingMeasurements(BerdyHelper& berdy,
                                          KinDynComputations& kinDynComputations,
                                          LinkPositions& linkPos,
                                          LinkVelArray&  linkVels,
                                          LinkAccArray&  linkProperAccs, //TODO check if prop acc
                                          LinkNetExternalWrenches& extWrenches,
                                          LinkIndex baseIdx)
{
    // Get link spatial inertias
    LinkInertias linkInertias(berdy.model());
    for(size_t linkIdx = 0; linkIdx < berdy.model().getNrOfLinks(); linkIdx++)
    {
        linkInertias(linkIdx) = berdy.model().getLink(linkIdx)->getInertia();
    }

    // Set gravitational wrench vector
    iDynTree::SpatialForceVector gravitationalWrenchInCentroidal;
    gravitationalWrenchInCentroidal.zero();
    gravitationalWrenchInCentroidal(2) = berdy.model().getTotalMass() * -9.81;

    iDynTree::Vector6 centroidalMomentum;
    centroidalMomentum.zero();

    iDynTree::Vector6 rocmInBaseRaw;
    rocmInBaseRaw.zero();

    // {}^B_X*dot_Gbar {}^Gbar_h
    iDynTree::Vector6 rocmInBaseBias;
    rocmInBaseBias.zero();

    // Set centroidal to world transform
    iDynTree::Transform world_H_centroidal = iDynTree::Transform::Identity();
    world_H_centroidal.setPosition(kinDynComputations.getCenterOfMassPosition());
    std::cerr<<"Center of mass position: "<<kinDynComputations.getCenterOfMassPosition().toString()<<std::endl;
    std::cerr<<"Base link idx: " <<baseIdx<<std::endl;

    // base_H_centroidal transform
    const iDynTree::Transform base_H_centroidal = kinDynComputations.getWorldBaseTransform().inverse() * world_H_centroidal;

    // Compute gravitational wrench expressed in base
    iDynTree::Vector6 gravitationalWrenchInBase;
    gravitationalWrenchInBase.zero();
    iDynTree::toEigen(gravitationalWrenchInBase) = iDynTree::toEigen(base_H_centroidal.asAdjointTransformWrench()) * iDynTree::toEigen(gravitationalWrenchInCentroidal);

    // Get base transform A_H_B
    const iDynTree::Transform world_H_base = linkPos(baseIdx);

    // Get base velocity A_v_A,B
    const iDynTree::Twist baseVelocityExpressedInWorld = linkVels(baseIdx);

    // Get base acceleration A_a_A,B
    const iDynTree::Twist baseAccelerationExpressedInWorld = linkProperAccs(baseIdx);

    // Get base velocity in base B_v_B
    iDynTree::Vector6 baseVelocityExpressedInBaseVector;
    iDynTree::toEigen(baseVelocityExpressedInBaseVector) = iDynTree::toEigen(world_H_base.inverse().asAdjointTransform()) * iDynTree::toEigen(baseVelocityExpressedInWorld);
    iDynTree::Twist baseVelocityExpressedInBase = Twist(SpatialVector<SpatialMotionVector>(baseVelocityExpressedInBaseVector));

    // Iterate over measurements
    for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < berdy.model().getNrOfLinks(); visitedLinkIndex++)
    {
        // Get world_H_link transform
        const iDynTree::Transform world_H_link = linkPos(visitedLinkIndex);

        // Get link to base transform
        const iDynTree::Transform base_H_link = world_H_base.inverse() * world_H_link;

        // Compute link to centroidal transform
        const iDynTree::Transform centroidal_H_link = world_H_centroidal.inverse() * world_H_link;

        // Get link velocity A_v_A,L
        const iDynTree::Twist linkVelocityExpressedInWorld = linkVels(visitedLinkIndex);

        // Get link acceleration A_a_A,L
        const iDynTree::Twist linkAccelerationExpressedInWorld = linkProperAccs(visitedLinkIndex);

        // Get L_v_A,L = L_X_A * L_v_A,L
        iDynTree::Vector6 linkVelocityExpressedInLink;
        {
            linkVelocityExpressedInLink.zero();
            iDynTree::toEigen(linkVelocityExpressedInLink) = iDynTree::toEigen(world_H_link.inverse().asAdjointTransform()) * iDynTree::toEigen(linkVelocityExpressedInWorld);
        }

        // Get L_v_B,L = (L_X_A * A_v_A,L) - (L_X_B * B_X_A * A_v_A,B) = L_X_A * (A_v_A,L - A_v_A,B)
        // from Eq (3.5a) from silvio's thesis https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf (Broken reference)
        iDynTree::Vector6 linkVelocityWrtBaseExpressedInLink;
        {
            linkVelocityWrtBaseExpressedInLink.zero();
            iDynTree::toEigen(linkVelocityWrtBaseExpressedInLink) = iDynTree::toEigen(world_H_link.inverse().asAdjointTransform()) * iDynTree::toEigen(linkVelocityExpressedInWorld - baseVelocityExpressedInWorld);
        }
        const iDynTree::Twist linkVelocityWrtBaseExpressedInLinkTwist = Twist(SpatialVector<SpatialMotionVector>(linkVelocityWrtBaseExpressedInLink));

        // Get L_a_A,L = L_X*_A * A_a_A,L 
        iDynTree::Vector6 linkAccelerationExpressedInLink;
        {
            linkAccelerationExpressedInLink.zero();
            iDynTree::toEigen(linkAccelerationExpressedInLink) = iDynTree::toEigen(world_H_link.inverse().asAdjointTransformWrench()) * iDynTree::toEigen(linkAccelerationExpressedInWorld);
            std::cerr<<std::endl<<"L_a_A,L:"
                     <<std::endl<<"L_X*_A: "<<std::endl<<world_H_link.inverse().asAdjointTransformWrench().toString()
                     <<std::endl<<"A_a_A,L: "<<linkAccelerationExpressedInWorld.toString()<<std::endl;
        }

        // Compute link momentum - G_X*_L * I_L * L_v_A,L
        iDynTree::Vector6 linkMomentum;
        {
            linkMomentum.zero();
            iDynTree::toEigen(linkMomentum) = iDynTree::toEigen(centroidal_H_link.asAdjointTransformWrench()) *
                                              iDynTree::toEigen(linkInertias(visitedLinkIndex).asMatrix()) *
                                              iDynTree::toEigen(linkVelocityExpressedInLink);

            //std::cerr<<std::endl<<"Link momentum for link "<<visitedLinkIndex<<": "<<std::endl<<
            //            "G_H*_L:"<< std::endl<<centroidal_H_link.asAdjointTransformWrench().toString()<<
            //            "L_I_L: "<< std::endl<<linkInertias(visitedLinkIndex).asMatrix().toString()<<
            //            "L_v_A,L: "<< std::endl<<linkVelocityExpressedInLink.toString()<<std::endl<<std::endl;
                    
        }

        // Update centroidal momentum
        iDynTree::toEigen(centroidalMomentum) += iDynTree::toEigen(linkMomentum);
        std::cerr<<"Link "<<visitedLinkIndex<<" momentum: "<<centroidalMomentum.toString()<<std::endl;

        // Compute link rate of change of momentum (expressed in base) term with accelerations -  B_X*_L * I_L * L_a_A,L
        //TODO is it okay with B_X*_L * I_L * L_a_A,L?? check inertia
        iDynTree::Vector6 linkROCMInBase_acc_term;
        iDynTree::toEigen(linkROCMInBase_acc_term) = iDynTree::toEigen(base_H_link.asAdjointTransformWrench()) *
                                                     iDynTree::toEigen(linkInertias(visitedLinkIndex).asMatrix()) *
                                                     iDynTree::toEigen(linkAccelerationExpressedInLink);

        std::cerr<<std::endl<<
        "Link rocm in base acceleration term:"<<std::endl<<
        "B_X*_L: "<<std::endl<<base_H_link.asAdjointTransformWrench().toString()<<std::endl<<
        "L_I_L: "<<std::endl<<linkInertias(visitedLinkIndex).asMatrix().toString()<<std::endl<<
        "L_a_A,L: "<<linkAccelerationExpressedInLink.toString()<<std::endl;
                    

        // Compute link rate of change of momentum (expressed in base) term with velocity - B_Xdot*_L * I_L * L_v_A,L
        iDynTree::Vector6 linkROCMInBase_vel_term;
        iDynTree::toEigen(linkROCMInBase_vel_term) = iDynTree::toEigen(base_H_link.asAdjointTransformWrench()) *
                                                      iDynTree::toEigen(linkVelocityWrtBaseExpressedInLinkTwist.asCrossProductMatrixWrench()) *
                                                      iDynTree::toEigen(linkInertias(visitedLinkIndex).asMatrix()) *
                                                      iDynTree::toEigen(linkVelocityExpressedInLink);

        // Update ROCM in base
        // Update rate of change of momentum
        iDynTree::toEigen(rocmInBaseRaw) += iDynTree::toEigen(linkROCMInBase_acc_term) +
                                        iDynTree::toEigen(linkROCMInBase_vel_term);

        std::cerr<<"ROCM: "<<
                    std::endl<<"ROCM in base: "<<rocmInBaseRaw.toString()<<
                    std::endl<<"Acc term: "<<linkROCMInBase_acc_term.toString()<<
                    std::endl<<"Vel term: "<<linkROCMInBase_vel_term.toString()<<std::endl;

    }



    // Compute position derivative (comVel - baseLinVel)
    iDynTree::Vector3 posDerivative;
    {
        posDerivative.setVal(0, kinDynComputations.getCenterOfMassVelocity().getVal(0) - baseVelocityExpressedInWorld.getLinearVec3().getVal(0));
        posDerivative.setVal(1, kinDynComputations.getCenterOfMassVelocity().getVal(1) - baseVelocityExpressedInWorld.getLinearVec3().getVal(1));
        posDerivative.setVal(2, kinDynComputations.getCenterOfMassVelocity().getVal(2) - baseVelocityExpressedInWorld.getLinearVec3().getVal(2));
    }
    
    // Compute rotation derivative ( base_dotR_world = ( Skew(baseAngVel) * world_R_base)' )
    iDynTree::Matrix3x3 rotDerivative;
    //iDynTree::toEigen(rotDerivative) = iDynTree::toEigen(world_H_base.getRotation()).transpose() *
    //                                   iDynTree::skew( iDynTree::toEigen(baseVelocityExpressedInWorld.getAngularVec3() ) ).transpose();

    iDynTree::toEigen(rotDerivative) = iDynTree::skew(-iDynTree::toEigen(baseVelocityExpressedInBase.getAngularVec3() ) ) * iDynTree::toEigen(base_H_centroidal.getRotation());
    
    std::cerr<<"ROT derivative: "<<rotDerivative.toString()<<std::endl;
    std::cerr<<"Skew :"<<iDynTree::skew(-iDynTree::toEigen(baseVelocityExpressedInWorld.getAngularVec3() ) )<<std::endl;
    std::cerr<<"Rot :"<<world_H_base.getRotation().toString()<<std::endl;

    iDynTree::TransformDerivative base_dotH_centroidal(rotDerivative, posDerivative);

    // Compute the bias term with centroidal momentum
    iDynTree::SpatialMomentum centroidalMom = iDynTree::SpatialMomentum(SpatialVector<SpatialForceVector>(centroidalMomentum));

    // Compute base_dotX*_centroidal * centroidal momentum
    iDynTree::SpatialForceVector biasTermFromCentroidalMomentum; 
    biasTermFromCentroidalMomentum = base_dotH_centroidal.transform(base_H_centroidal, centroidalMom);

    std::cerr<<"Computed ROCM:"<<
                std::endl<<"ROCM in base: "<<rocmInBaseRaw.toString()<<
                std::endl<<"Bias term: "<<biasTermFromCentroidalMomentum.toString()<<
                "Gravitational wrench in base:"<<gravitationalWrenchInBase.toString()<<std::endl;
    
    //TODO check this
    gravitationalWrenchInBase.zero();

    // Computed rate of change of momentum
    Vector6 rocmInBase;
    iDynTree::toEigen(rocmInBase) = iDynTree::toEigen(rocmInBaseRaw)
                                    - iDynTree::toEigen(biasTermFromCentroidalMomentum.asVector())
                                    - iDynTree::toEigen(gravitationalWrenchInBase);

    return rocmInBase;
}

void testBerdySensorMatrices(BerdyHelper & berdy, std::string filename)
{
    // Check the concistency of the sensor matrices
    // Generate a random pos, vel, acc, external wrenches
    FreeFloatingPos pos(berdy.model());
    FreeFloatingVel vel(berdy.model());
    FreeFloatingAcc generalizedProperAccs(berdy.model());
    LinkNetExternalWrenches extWrenches(berdy.model());

    getRandomInverseDynamicsInputsCustom(pos,vel,generalizedProperAccs,extWrenches);

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
    
    LinkAccArray linkAccs(berdy.model());
    for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < berdy.model().getNrOfLinks(); visitedLinkIndex++)
    {
        linkAccs(visitedLinkIndex).setLinearVec3(linkProperAccs(visitedLinkIndex).getLinearVec3());
        linkAccs(visitedLinkIndex).setAngularVec3(linkProperAccs(visitedLinkIndex).getAngularVec3());
        //TODO gravity?
        //linkAccs(visitedLinkIndex).getLinearVec3()(2) += -9.81;
    }

    RNEADynamicPhase(berdy.model(),berdy.dynamicTraversal(),
                     pos.jointPos(),linkVels,linkAccs,//linkAccs,linkProperAccs
                     extWrenches,intWrenches,genTrqs);

    // Propagate kinematics also inside berdy

    std::cerr<<"LINK PROP ACC: "<< linkProperAccs.toString(berdy.model());
    std::cerr<<"LINK accelerations used: "<< linkProperAccs.toString(berdy.model());
    std::cerr<<"Link vels: "<<linkVels.toString(berdy.model());
    std::cerr<<"Link positions: "<<std::endl<<linkPos.toString(berdy.model());

    // Correct for the unconsistency between the input net wrenches and the residual of the RNEA
    extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex()) = extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())+genTrqs.baseWrench();
    // extWrenches(berdy.dynamicTraversal().getBaseLink()->getIndex())(2) = 9.81;
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

    std::cerr<<"Ext wrenches are: "<<std::endl<<extWrenches.toString(berdy.model());

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

    // Variant BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES does not have D and bD
    if(berdy.getOptions().berdyVariant != BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES)
    {
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
    }

    if( berdy.getNrOfSensorsMeasurements() > 0 )
    {
        std::cout << "BerdyHelperUnitTest, testing sensors matrix for model " << filename <<  std::endl;

        // Generate the y vector of sensor measurements using the predictSensorMeasurements function
        VectorDynSize y(berdy.getNrOfSensorsMeasurements());
        y.zero();
        SensorsMeasurements sensMeas(berdy.sensors());
        bool ok = predictSensorsMeasurementsFromRawBuffers(berdy.model(),berdy.sensors(),berdy.dynamicTraversal(),
                                                           linkVels,linkAccs,intWrenches,sensMeas);
 
        Vector6 rocm;
        rocm.zero();
        if(berdy.getOptions().includeROCMAsSensor)
        {
            // set kyndyncomputations object for ROCM computation
            KinDynComputations kinDynComputations;
            kinDynComputations.loadRobotModel(berdy.model());

            Vector3 worldGravityAcc;
            worldGravityAcc.zero();
            //worldGravityAcc[2] = -9.81;

            if(true)
            {
                kinDynComputations.setRobotState(linkPos(baseIdx), //w_T_base
                                                pos.jointPos(), //joint positions
                                                linkVels(baseIdx), //base velocity
                                                vel.jointVel(), // joint velocities
                                                worldGravityAcc); // world gravity
            }

            rocm = computeROCMInBaseUsingMeasurements(berdy,
                                                      kinDynComputations,
                                                      linkPos,
                                                      linkVels,
                                                      linkAccs,
                                                      extWrenches,
                                                      baseIdx);
            
            std::cerr<<std::endl<<"Rocm is "<<rocm.toString()<<std::endl<<std::endl;
        }

        ASSERT_IS_TRUE(ok);
        ok = berdy.serializeSensorVariables(sensMeas,extWrenches,genTrqs.jointTorques(),generalizedProperAccs.jointAcc(),intWrenches,rocm,y);
        ASSERT_IS_TRUE(ok);

        // Check that y = Y*d + bY
        VectorDynSize yFromBerdy(berdy.getNrOfSensorsMeasurements());

        ASSERT_EQUAL_DOUBLE(berdy.getNrOfSensorsMeasurements(), Y.rows());
        ASSERT_EQUAL_DOUBLE(berdy.getNrOfSensorsMeasurements(), bY.size());

        toEigen(yFromBerdy) = toEigen(Y)*toEigen(d) + toEigen(bY);

        //std::cerr << "Y : " << std::endl;
        //std::cerr << Y.description(true) << std::endl;
        //std::cerr << "d :\n" << d.toString() << std::endl;
        
        std::cerr << "Y*d :\n" << toEigen(Y)*toEigen(d) << std::endl;
        std::cerr << "bY :\n" << bY.toString() << std::endl;
        std::cerr << "y from model:\n" << y.toString() << std::endl;
        

        // Check if the two vectors are equal
        std::cerr << "Testing y and yFromBerdy" << std::endl;
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
        Vector6 dummyRocm;
        dummyRocm.zero();
        ok = berdy.serializeSensorVariables(sensMeas,extWrenches,genTrqs.jointTorques(),generalizedProperAccs.jointAcc(),intWrenches,dummyRocm,y);
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

    /*// We test the floating base BERDY
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
*/
    // We test the floating base BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES
    options.berdyVariant = iDynTree::BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES;
    // Include Rate Of Change of Momentum
    options.includeROCMAsSensor = true;
    options.includeAllJointTorquesAsSensors = false;
    options.includeAllJointAccelerationsAsSensors = false;
    options.includeAllNetExternalWrenchesAsSensors = true;
    //options.jointOnWhichTheInternalWrenchIsMeasured.clear();
    ok = berdyHelper.init(estimator.model(), estimator.sensors(), options);
    ASSERT_IS_TRUE(ok);
    testBerdySensorMatrices(berdyHelper, fileName);
    std::cerr<<"OK DONE"<<std::endl;ASSERT_IS_TRUE(false);
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
