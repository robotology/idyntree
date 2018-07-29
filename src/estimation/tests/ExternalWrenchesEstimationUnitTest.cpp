/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>
#include <iDynTree/Sensors/PredictSensorsMeasurements.h>

#include <iDynTree/Model/ContactWrench.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/ModelTestUtils.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/TestUtils.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

inline Link getSimplestLink()
{
    double cxx = 0.5;
    double cyy = 1.0;
    double czz = 6.7;
    double rotInertiaData[3*3] = {czz+cyy,0.0,0.0,
                                  0.0,cxx+czz,0.0,
                                  0.0,0.0,cxx+cyy};

    Rotation rot = Rotation::RPY(2.0,5.0,5.0);

    SpatialInertia inertiaLink(7.0,
                               Position(5.0,0.0,0.0),
                               rot*RotationalInertiaRaw(rotInertiaData,3,3));

    Link link;

    link.setInertia(inertiaLink);

    return link;
}

inline Model getSimplestModel(unsigned int nrOfJoints)
{
    Model model;

    model.addLink("baseLink",getSimplestLink());

    return model;
}

void checkSimpleModelExternalWrenchEstimation(size_t nrOfJoints)
{
    // Create a random one link model
    Model model = getSimplestModel(0);

    // Use default traversal
    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Generate random position, velocities and accelerations
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);
    FreeFloatingAcc robotAcc(model);

    robotPos.worldBasePos() = getRandomTransform();
    getRandomVector(robotPos.jointPos());

    robotVel.baseVel() = getRandomTwist();
    getRandomVector(robotVel.jointVel());

    robotAcc.baseAcc() = getRandomTwist();
    getRandomVector(robotAcc.jointAcc());

    LinkVelArray vels(model);
    LinkAccArray properAccs(model);

    ForwardVelAccKinematics(model,traversal,robotPos,robotVel,robotAcc,vels,properAccs);

    // Let's compute the external wrench acting on a body
    LinkUnknownWrenchContacts unknownWrenches(model);

    // Add an unknownWrench
    UnknownWrenchContact unknownWrench;
    unknownWrench.unknownType = FULL_WRENCH;
    unknownWrench.contactPoint = getRandomPosition();
    LinkIndex contactLink = getRandomLinkIndexOfModel(model);
    unknownWrenches.setNrOfContactsForLink(contactLink,1);
    unknownWrenches.contactWrench(contactLink,0) = unknownWrench;

    LinkContactWrenches contactWrenches(model);

    estimateExternalWrenchesBuffers bufs(1,model.getNrOfLinks());
    estimateExternalWrenchesWithoutInternalFT(model,traversal,unknownWrenches,robotPos.jointPos(),vels,properAccs,bufs,contactWrenches);


    // Let's compute the new contact wrenches
    LinkNetExternalWrenches newContactWrenches(model);
    LinkInternalWrenches internalWrenches(model);
    FreeFloatingGeneralizedTorques trqs(model);

    contactWrenches.computeNetWrenches(newContactWrenches);

    // Let's compute the dynamics pass of RNEA
    // given that the external force on a link has been consistently
    // with the proper accelerations, the base wrench compute by RNEA
    // should be 0
    RNEADynamicPhase(model,traversal,robotPos.jointPos(),vels,properAccs,newContactWrenches,internalWrenches,trqs);
}

void checkRandomModelExternalWrenchEstimation(size_t nrOfJoints)
{
    std::cerr << "Check random model with " << nrOfJoints << " joints." << std::endl;

    // Create a random one link model
    Model model = getRandomModel(nrOfJoints);

    // Use default traversal
    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Generate random position, velocities and accelerations
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);
    FreeFloatingAcc robotAcc(model);

    robotPos.worldBasePos() = getRandomTransform();
    getRandomVector(robotPos.jointPos());

    robotVel.baseVel() = getRandomTwist();
    getRandomVector(robotVel.jointVel());

    robotAcc.baseAcc() = getRandomTwist();
    getRandomVector(robotAcc.jointAcc());

    LinkVelArray vels(model);
    LinkAccArray properAccs(model);

    ForwardVelAccKinematics(model,traversal,robotPos,robotVel,robotAcc,vels,properAccs);

    // Let's compute the external wrench acting on a body
    LinkUnknownWrenchContacts unknownWrenches(model);

    // Add an unknownWrench
    UnknownWrenchContact unknownWrench;
    unknownWrench.unknownType = FULL_WRENCH;
    unknownWrench.contactPoint = getRandomPosition();
    LinkIndex contactLink = getRandomLinkIndexOfModel(model);
    unknownWrenches.addNewContactForLink(contactLink,unknownWrench);

    UnknownWrenchContact unknownWrench2;
    unknownWrench2.unknownType = FULL_WRENCH;
    unknownWrench2.contactPoint = getRandomPosition();
    LinkIndex contactLink2 = getRandomLinkIndexOfModel(model);
    unknownWrenches.addNewContactForLink(contactLink2,unknownWrench2);


    LinkContactWrenches contactWrenches(model);

    estimateExternalWrenchesBuffers bufs(1,model.getNrOfLinks());
    estimateExternalWrenchesWithoutInternalFT(model,traversal,unknownWrenches,robotPos.jointPos(),vels,properAccs,bufs,contactWrenches);

    // Let's compute the new contact wrenches
    LinkNetExternalWrenches newContactWrenches(model);
    LinkInternalWrenches internalWrenches(model);
    FreeFloatingGeneralizedTorques trqs(model);

    contactWrenches.computeNetWrenches(newContactWrenches);

    // Let's compute the dynamics pass of RNEA
    // given that the external force on a link has been consistently
    // with the proper accelerations, the base wrench compute by RNEA
    // should be 0
    RNEADynamicPhase(model,traversal,robotPos.jointPos(),vels,properAccs,newContactWrenches,internalWrenches,trqs);

    SpatialForceVector zero = SpatialForceVector::Zero();
    ASSERT_EQUAL_SPATIAL_FORCE(trqs.baseWrench(),zero);

    // Once we computed a resonable force, we simulate some ft sensors measures
}

void checkSimpleModelExternalWrenchEstimationWithFTSensors()
{
    std::cerr << "checkSimpleModelExternalWrenchEstimationWithFTSensors " << std::endl;

    // Create a simple three link model
    double rotInertiaData[3*3] = {1.0,0.0,0.0,
                                  0.0,1.0,0.0,
                                  0.0,0.0,1.0};

    SpatialInertia inertia(1.0,iDynTree::Position::Zero(),RotationalInertiaRaw(rotInertiaData,3,3));
    Link link0, link1, link2, link3;
    link0.setInertia(inertia);
    link1.setInertia(inertia);
    link2.setInertia(inertia);
    link3.setInertia(inertia);
    Model model;
    model.addLink("link0",link0);
    model.addLink("link1",link1);
    model.addLink("link2",link2);
    model.addLink("link3",link3);

    FixedJoint joint01(0,1,iDynTree::Transform::Identity());
    FixedJoint joint12(1,2,iDynTree::Transform::Identity());
    Axis axis;
    Direction dir(1.0,0.0,0.0);
    axis.setDirection(dir);
    axis.setOrigin(Position::Zero());
    RevoluteJoint joint23;
    joint23.setAttachedLinks(2, 3);
    joint23.setRestTransform(iDynTree::Transform::Identity());
    joint23.setAxis(axis, 3);

    model.addJoint("joint01",&joint01);
    model.addJoint("joint12",&joint12);
    model.addJoint("joint23",&joint23);

    std::vector<std::string> jointNames;
    jointNames.push_back("joint01");
    jointNames.push_back("joint12");

    // Create sensors
    SixAxisForceTorqueSensor ft01;
    ft01.setName("ft01");
    ft01.setParentJoint("joint01");
    ft01.setParentJointIndex(0);
    ft01.setFirstLinkName(model.getLinkName(0));
    ft01.setFirstLinkSensorTransform(0,Transform::Identity());
    ft01.setSecondLinkName(model.getLinkName(1));
    ft01.setSecondLinkSensorTransform(1,Transform::Identity());
    ft01.setAppliedWrenchLink(0);

    SixAxisForceTorqueSensor ft12;
    ft12.setName("ft12");
    ft12.setParentJoint("joint12");
    ft12.setParentJointIndex(1);
    ft12.setFirstLinkName(model.getLinkName(1));
    ft12.setFirstLinkSensorTransform(1,Transform::Identity());
    ft12.setSecondLinkName(model.getLinkName(2));
    ft12.setSecondLinkSensorTransform(2,Transform::Identity());
    ft12.setAppliedWrenchLink(1);

    SensorsList sensors;
    sensors.addSensor(ft01);
    sensors.addSensor(ft12);

    // Set the measured sensors
    SensorsMeasurements measSens(sensors);
    SensorsMeasurements simSens(sensors);

    measSens.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,0,getRandomWrench());
    measSens.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,1,getRandomWrench());


    // Use default traversal
    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Generate random position, velocities and accelerations
    FreeFloatingPos robotPos(model);
    FreeFloatingVel robotVel(model);
    FreeFloatingAcc robotAcc(model);

    robotPos.worldBasePos() = getRandomTransform();
    getRandomVector(robotPos.jointPos());

    robotVel.baseVel() = getRandomTwist();
    getRandomVector(robotVel.jointVel());

    robotAcc.baseAcc() = getRandomTwist();
    robotAcc.baseAcc()(2) = 10.0;
    getRandomVector(robotAcc.jointAcc());

    LinkVelArray vels(model);
    LinkAccArray properAccs(model);

    std::cerr << "before vels" << vels.toString(model) << std::endl;

    ForwardVelAccKinematics(model,traversal,robotPos,robotVel,robotAcc,vels,properAccs);

    std::cerr << "after vels" << vels.toString(model) << std::endl;

    // Let's compute the external wrench acting on a body
    LinkUnknownWrenchContacts unknownWrenches(model);

    // Add an unknownWrench for each link
    UnknownWrenchContact unknownWrench;
    unknownWrench.unknownType = FULL_WRENCH;
    unknownWrench.contactPoint = iDynTree::Position::Zero();

    LinkIndex contactLink = 0;
    unknownWrenches.setNrOfContactsForLink(contactLink,1);
    unknownWrenches.contactWrench(contactLink,0) = unknownWrench;

    contactLink = 1;
    unknownWrenches.setNrOfContactsForLink(contactLink,1);
    unknownWrenches.contactWrench(contactLink,0) = unknownWrench;

    contactLink = 2;
    unknownWrenches.setNrOfContactsForLink(contactLink,1);
    unknownWrenches.contactWrench(contactLink,0) = unknownWrench;

    // Let's resize the contact wrenches
    LinkContactWrenches contactWrenches(model);

    // Let's compute the submodel decomposition
    SubModelDecomposition subModels;
    subModels.splitModelAlongJoints(model,traversal,jointNames);

    // Let's estimate external forces
    estimateExternalWrenchesBuffers bufs(subModels);
    std::cerr << "vels" << vels.toString(model) << std::endl;


    estimateExternalWrenches(model,subModels,sensors,unknownWrenches,robotPos.jointPos(),vels,properAccs,measSens,bufs,contactWrenches);

    // Let's compute the new contact wrenches
    LinkNetExternalWrenches newContactWrenches(model);
    LinkInternalWrenches internalWrenches(model);
    FreeFloatingGeneralizedTorques trqs(model);

    std::cerr << "Contact wrenches" << contactWrenches.toString(model) << std::endl;

    contactWrenches.computeNetWrenches(newContactWrenches);

    std::cerr << "New contact wrenches" << newContactWrenches.toString(model) << std::endl;


    // Let's compute the dynamics pass of RNEA
    // given that the external force on a link has been consistently
    // with the proper accelerations, the base wrench compute by RNEA
    // should be 0
    RNEADynamicPhase(model,traversal,robotPos.jointPos(),vels,properAccs,newContactWrenches,internalWrenches,trqs);

    std::cerr << "Internal wrench" << internalWrenches.toString(model) << std::endl;

    // Let's simulate measurements
    iDynTree::SensorsMeasurements simulatedSensors(sensors);
    iDynTree::predictSensorsMeasurementsFromRawBuffers(model,sensors,traversal,
                                                       vels,properAccs,internalWrenches,
                                                       simulatedSensors);

    for(size_t simFT=0; simFT < simulatedSensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); simFT++)
    {
        iDynTree::Wrench simWrench;
        simulatedSensors.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,simFT,simWrench);
        std::cerr << "Simulated measurement : of sensor " << sensors.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,simFT)->getName()
                  << " is " << simWrench.toString() << std::endl;
        iDynTree::Wrench actualWrench;
        measSens.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,simFT,actualWrench);
        std::cerr << "Actual measurement : of sensor " << sensors.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,simFT)->getName()
                  << " is " << actualWrench.toString() << std::endl;
    }


    SpatialForceVector zero = SpatialForceVector::Zero();
    ASSERT_EQUAL_SPATIAL_FORCE(trqs.baseWrench(),zero);


}


int main()
{
    std::cerr << "Checking the simplest model: " << std::endl;
    checkSimpleModelExternalWrenchEstimation(0);

    checkRandomModelExternalWrenchEstimation(0);
    checkRandomModelExternalWrenchEstimation(1);
    checkRandomModelExternalWrenchEstimation(2);
    checkRandomModelExternalWrenchEstimation(3);
    checkRandomModelExternalWrenchEstimation(10);
    checkRandomModelExternalWrenchEstimation(20);

    checkSimpleModelExternalWrenchEstimationWithFTSensors();

    return EXIT_SUCCESS;
}
