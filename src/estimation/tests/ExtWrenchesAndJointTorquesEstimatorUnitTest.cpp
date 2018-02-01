/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>


using namespace iDynTree;

std::vector<std::string> getCanonical_iCubJoints()
{
    std::vector<std::string> consideredJoints;

    consideredJoints.push_back("torso_pitch");
    consideredJoints.push_back("torso_roll");
    consideredJoints.push_back("torso_yaw");
    consideredJoints.push_back("neck_pitch");
    consideredJoints.push_back("neck_roll");
    consideredJoints.push_back("neck_yaw");
    consideredJoints.push_back("l_shoulder_pitch");
    consideredJoints.push_back("l_shoulder_roll");
    consideredJoints.push_back("l_shoulder_yaw");
    consideredJoints.push_back("l_elbow");
    consideredJoints.push_back("r_shoulder_pitch");
    consideredJoints.push_back("r_shoulder_roll");
    consideredJoints.push_back("r_shoulder_yaw");
    consideredJoints.push_back("r_elbow");
    consideredJoints.push_back("l_hip_pitch");
    consideredJoints.push_back("l_hip_roll");
    consideredJoints.push_back("l_hip_yaw");
    consideredJoints.push_back("l_knee");
    consideredJoints.push_back("l_ankle_pitch");
    consideredJoints.push_back("l_ankle_roll");
    consideredJoints.push_back("r_hip_pitch");
    consideredJoints.push_back("r_hip_roll");
    consideredJoints.push_back("r_hip_yaw");
    consideredJoints.push_back("r_knee");
    consideredJoints.push_back("r_ankle_pitch");
    consideredJoints.push_back("r_ankle_roll");

    return consideredJoints;
}

void setDOFsSubSetPositionsInDegrees(const iDynTree::Model & model,
                                     const std::vector<std::string> & dofNames,
                                     const std::vector<double> & dofPositionsInDegrees,
                                     JointPosDoubleArray & qj)
{
    for(int i=0; i < dofNames.size(); i++)
    {
        iDynTree::JointIndex jntIdx = model.getJointIndex(dofNames[i]);
        size_t dofOffset = model.getJoint(jntIdx)->getDOFsOffset();
        qj(dofOffset) = deg2rad(dofPositionsInDegrees[i]);
    }
}


int main()
{

    std::cerr << "Test ft offset estimation on the iCubDarmstadt01 model: " << std::endl;

    // We will compare the estimatas obtained using
    // the IMU measurement and the fixed base measurements
    ExtWrenchesAndJointTorquesEstimator estimatorIMU, estimatorFixedBase;

    std::vector<std::string> consideredJoints = getCanonical_iCubJoints();

    estimatorIMU.loadModelAndSensorsFromFileWithSpecifiedDOFs(getAbsModelPath("iCubDarmstadt01.urdf"),consideredJoints);
    estimatorFixedBase.setModelAndSensors(estimatorIMU.model(),estimatorIMU.sensors());

    ASSERT_EQUAL_DOUBLE(estimatorIMU.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),6);

    JointPosDoubleArray qj(estimatorIMU.model());
    JointDOFsDoubleArray dqj(estimatorIMU.model()), ddqj(estimatorIMU.model());

    qj.zero();
    dqj.zero();
    ddqj.zero();

    // Set a desired configuration
    std::vector<std::string> dofNames;
    std::vector<double> dofPositionsInDegrees;

    dofNames.push_back("l_shoulder_pitch");
    dofPositionsInDegrees.push_back(-90.0);
    dofNames.push_back("l_shoulder_roll");
    dofPositionsInDegrees.push_back(15.0);
    dofNames.push_back("l_shoulder_yaw");
    dofPositionsInDegrees.push_back(0);
    dofNames.push_back("l_elbow");
    dofPositionsInDegrees.push_back(90.0);

    dofNames.push_back("r_shoulder_pitch");
    dofPositionsInDegrees.push_back(-90.0);
    dofNames.push_back("r_shoulder_roll");
    dofPositionsInDegrees.push_back(15.0);
    dofNames.push_back("r_shoulder_yaw");
    dofPositionsInDegrees.push_back(0);
    dofNames.push_back("r_elbow");
    dofPositionsInDegrees.push_back(90.0);


    setDOFsSubSetPositionsInDegrees(estimatorIMU.model(),dofNames,dofPositionsInDegrees,qj);

    Vector3 gravityOnRootLink;
    gravityOnRootLink.zero();
    gravityOnRootLink(2) = -9.81;

    Vector3 properAccelerationInIMU;
    properAccelerationInIMU.zero();
    properAccelerationInIMU(2) = 9.81;

    Vector3 zero;
    zero.zero();

    FrameIndex root_link_index = estimatorFixedBase.model().getFrameIndex("root_link");
    FrameIndex imu_frame_index = estimatorIMU.model().getFrameIndex("imu_frame");

    // Set kinematics
    estimatorFixedBase.updateKinematicsFromFixedBase(qj,dqj,ddqj,root_link_index,gravityOnRootLink);
    estimatorIMU.updateKinematicsFromFloatingBase(qj,dqj,ddqj,imu_frame_index,properAccelerationInIMU,zero,zero);

    // Compute ft sensor offset
    UnknownWrenchContact unknown;
    unknown.unknownType = iDynTree::FULL_WRENCH;
    unknown.contactPoint.zero();

    LinkUnknownWrenchContacts fullBodyUnknowns(estimatorIMU.model());

    fullBodyUnknowns.addNewContactInFrame(estimatorIMU.model(),root_link_index,unknown);

    SensorsMeasurements sensOffsetIMU(estimatorIMU.sensors());
    SensorsMeasurements sensOffsetFixedBase(estimatorFixedBase.sensors());

    LinkContactWrenches estimatedContactWrenchesIMU(estimatorIMU.model());
    LinkContactWrenches estimatedContactWrenchesFixedBase(estimatorFixedBase.model());

    JointDOFsDoubleArray estimatedJointTorquesIMU(estimatorIMU.model());
    JointDOFsDoubleArray estimatedJointTorquesFixedBase(estimatorFixedBase.model());

    estimatorIMU.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,sensOffsetIMU,estimatedContactWrenchesIMU,estimatedJointTorquesIMU);
    estimatorFixedBase.computeExpectedFTSensorsMeasurements(fullBodyUnknowns,sensOffsetFixedBase,estimatedContactWrenchesFixedBase,estimatedJointTorquesFixedBase);

    for(unsigned int ft=0; ft < sensOffsetIMU.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++)
    {
        iDynTree::Wrench ftIMU, ftFixedBase;


        sensOffsetIMU.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,ftIMU);
        sensOffsetFixedBase.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,ftFixedBase);

        std::cerr << "Wrench for sensor " << estimatorIMU.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName()
                  << " are " << ftIMU.toString() << " (IMU) " << ftFixedBase.toString() << " (fixed base)" << std::endl;

        ASSERT_EQUAL_SPATIAL_FORCE_TOL(ftIMU,ftFixedBase,1e-9);
    }

    // Add a test on net external wrenches
    // Compute the net wrenches acting on a link
    LinkNetTotalWrenchesWithoutGravity netWrenchesWithoutGravity(estimatorIMU.model());
    bool ok = estimatorIMU.estimateLinkNetWrenchesWithoutGravity(netWrenchesWithoutGravity);
    ASSERT_IS_TRUE(ok);

    // Compute the external wrenches
    LinkNetExternalWrenches externalWrenches(estimatorIMU.model());
    ok = estimatedContactWrenchesFixedBase.computeNetWrenches(externalWrenches);
    ASSERT_IS_TRUE(ok);


    LinkPositions base_H_link(estimatorIMU.model());
    // Compute the transform from each link to the base
    Traversal defaultTraversal;
    estimatorIMU.model().computeFullTreeTraversal(defaultTraversal);
    ok = iDynTree::ForwardPositionKinematics(estimatorIMU.model(),
                                             defaultTraversal,
                                             Transform::Identity(),
                                             qj,base_H_link);

    iDynTree::Wrench momentumDerivative;
    momentumDerivative.zero();
    iDynTree::Wrench externalForces;
    externalForces.zero();

    for(LinkIndex idx = 0; idx < estimatorIMU.model().getNrOfLinks(); idx++)
    {
        momentumDerivative = momentumDerivative + base_H_link(idx)*netWrenchesWithoutGravity(idx);
        externalForces     = externalForces + base_H_link(idx)*externalWrenches(idx);
    }

    // The sum of the netWrenchesWithoutGravity of all the links should be equal to the sum of the external wrenches
    ASSERT_EQUAL_SPATIAL_FORCE_TOL(momentumDerivative,externalForces,1e-8);


    return EXIT_SUCCESS;
}
