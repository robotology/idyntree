/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include "testModels.h"


using namespace iDynTree;

int main()
{

    std::cerr << "Test ft offset estimation on the iCubGenova02 model: " << std::endl;
    ExtWrenchesAndJointTorquesEstimator estimator;
    estimator.loadModelAndSensorsFromFile(getAbsModelPath("iCubGenova02.urdf"));

    JointPosDoubleArray qj(estimator.model());
    JointDOFsDoubleArray dqj(estimator.model()), ddqj(estimator.model());

    qj.zero();
    dqj.zero();
    ddqj.zero();

    Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.8;

    FrameIndex l_sole_index = estimator.model().getFrameIndex("l_sole");

    // Set kinematics
    estimator.updateKinematicsFromFixedBase(qj,dqj,ddqj,l_sole_index,gravity);

    // Compute ft sensor offset
    UnknownWrenchContact unknown;
    unknown.unknownType = iDynTree::FULL_WRENCH;
    unknown.contactPoint.zero();

    LinkUnknownWrenchContacts fullBodyUnknowns(estimator.model());

    fullBodyUnknowns.addNewContactInFrame(estimator.model(),l_sole_index,unknown);

    SensorsMeasurements sensOffset(estimator.sensors());

    LinkContactWrenches estimatedContactWrenches(estimator.model());
    JointDOFsDoubleArray estimatedJointTorques(estimator.model());

    estimator.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,sensOffset,estimatedContactWrenches,estimatedJointTorques);

    std::cerr << "Estimated contact wrenches " << estimatedContactWrenches.toString(estimator.model()) << std::endl;
    std::cerr << "Estimated joint torques " << estimatedJointTorques.toString() << std::endl;


    return EXIT_SUCCESS;
}
