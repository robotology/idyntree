/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * website: www.icub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include <iDynTree/Estimation/ExternalWrenchesEstimation.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenMathHelpers.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/ClassicalAcc.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/ContactWrench.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/ModelTransformers.h>
#include <iDynTree/Sensors/ModelSensorsTransformers.h>

#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>
#include <iDynTree/Sensors/PredictSensorsMeasurements.h>

#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFGenericSensorsImport.h>

namespace iDynTree
{

ExtWrenchesAndJointTorquesEstimator::ExtWrenchesAndJointTorquesEstimator():
    m_model(),
    m_submodels(),
    m_sensors(),
    m_isModelValid(false),
    m_isKinematicsUpdated(false),
    m_dynamicTraversal(),
    m_kinematicTraversals(),
    m_jointPos(),
    m_linkVels(),
    m_linkProperAccs(),
    m_linkIntWrenches(),
    m_linkNetExternalWrenches(),
    m_generalizedTorques(),
    m_calibBufs(),
    m_bufs()
{

}

ExtWrenchesAndJointTorquesEstimator::~ExtWrenchesAndJointTorquesEstimator()
{
    freeKinematicTraversals();
}


void getFTJointNames(const SensorsList & _sensors,
                     std::vector<std::string> ftJointNames)
{
    ftJointNames.resize(0);

    for(size_t sens=0; sens < _sensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE); sens++)
    {
        SixAxisForceTorqueSensor* pSens = static_cast<SixAxisForceTorqueSensor*>(_sensors.getSensor(SIX_AXIS_FORCE_TORQUE,sens));
        ftJointNames.push_back(pSens->getParentJoint());
    }

    return;
}

void ExtWrenchesAndJointTorquesEstimator::allocKinematicTraversals(const size_t nrOfLinks)
{
    m_kinematicTraversals.resize(nrOfLinks);
    for(size_t link=0; link < m_kinematicTraversals.size(); link++)
    {
        m_kinematicTraversals[link] = new Traversal();
    }
}

void ExtWrenchesAndJointTorquesEstimator::freeKinematicTraversals()
{
    for(size_t link=0; link < m_kinematicTraversals.size(); link++)
    {
        delete m_kinematicTraversals[link];
    }
    m_kinematicTraversals.resize(0);
}


bool ExtWrenchesAndJointTorquesEstimator::setModelAndSensors(const Model& _model,
                                                             const SensorsList& _sensors)
{
    // \todo TODO add isConsistent methods to Model and SensorList class


    m_model = _model;
    m_sensors = _sensors;

    // resize the data structures
    m_model.computeFullTreeTraversal(m_dynamicTraversal);
    freeKinematicTraversals();
    allocKinematicTraversals(m_model.getNrOfLinks());


    m_linkVels.resize(m_model);
    m_linkProperAccs.resize(m_model);
    m_linkIntWrenches.resize(m_model);
    m_linkNetExternalWrenches.resize(m_model);
    m_generalizedTorques.resize(m_model);

    // create submodel structure
    std::vector<std::string> ftJointNames;
    getFTJointNames(m_sensors,ftJointNames);
    bool ok = m_submodels.splitModelAlongJoints(m_model,m_dynamicTraversal,ftJointNames);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","setModelAndSensors","Error in creating submodel decomposition of the model.");
        return false;
    }

    m_bufs.resize(m_submodels);
    m_calibBufs.resize(1,_model.getNrOfLinks());

    // set that the model is valid
    m_isModelValid = true;

    return true;
}

bool ExtWrenchesAndJointTorquesEstimator::loadModelAndSensorsFromFile(const std::string filename,
                                                                      const std::string /*filetype*/)
{
    Model _model;
    SensorsList _sensors;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDF(filename,_model);

    if( !parsingCorrect )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","loadModelAndSensorsFromFile","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDF(filename,_model,_sensors);

    if( !parsingCorrect )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","loadModelAndSensorsFromFile","Error in parsing sensors from URDF.");
        return false;
    }

    return setModelAndSensors(_model,_sensors);
}

bool ExtWrenchesAndJointTorquesEstimator::loadModelAndSensorsFromFileWithSpecifiedDOFs(const std::string filename,
                                                                                       const std::vector< std::string >& consideredDOFs,
                                                                                       const std::string filetype)
{
    Model _modelFull;
    SensorsList _sensorsFull;

    bool parsingCorrect = false;

    parsingCorrect = modelFromURDF(filename,_modelFull);

    if( !parsingCorrect )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","loadModelAndSensorsFromFileWithSpecifiedDOFs","Error in parsing model from URDF.");
        return false;
    }

    parsingCorrect = sensorsFromURDF(filename,_modelFull,_sensorsFull);

    if( !parsingCorrect )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","loadModelAndSensorsFromFileWithSpecifiedDOFs","Error in parsing sensors from URDF.");
        return false;
    }

    // We need to create a reduced model, inclusing only the consideredDOFs and the fixed joints used by the FT sensors
    std::vector< std::string > consideredJoints = consideredDOFs;

    // Add FT fixed joints
    std::vector< std::string > ftJointNames;
    getFTJointNames(_sensorsFull,ftJointNames);

    for(size_t i=0; i < ftJointNames.size(); i++)
    {
        consideredJoints.push_back(ftJointNames[i]);
    }


    Model _modelReduced;
    SensorsList _sensorsReduced;

    //
    iDynTree::createReducedModelAndSensors(_modelFull,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

    return setModelAndSensors(_modelReduced,_sensorsReduced);
}


const Model& ExtWrenchesAndJointTorquesEstimator::model() const
{
    return m_model;
}

const SensorsList& ExtWrenchesAndJointTorquesEstimator::sensors() const
{
    return m_sensors;
}

const SubModelDecomposition& ExtWrenchesAndJointTorquesEstimator::submodels() const
{
    return m_submodels;
}


bool ExtWrenchesAndJointTorquesEstimator::updateKinematicsFromFixedBase(const JointPosDoubleArray& jointPos,
                                                                        const JointDOFsDoubleArray& jointVel,
                                                                        const JointDOFsDoubleArray& jointAcc,
                                                                        const FrameIndex& fixedFrame,
                                                                        const Vector3& gravity)
{
    if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFixedBase","Model and sensors information not setted.");
        return false;
    }

    Vector3 zero;
    zero.zero();

    Vector3 properClassicalAcceleration;

    properClassicalAcceleration(0) = -gravity(0);
    properClassicalAcceleration(1) = -gravity(1);
    properClassicalAcceleration(2) = -gravity(2);

    return updateKinematicsFromFloatingBase(jointPos,jointVel,jointAcc,fixedFrame,properClassicalAcceleration,zero,zero);
}

bool ExtWrenchesAndJointTorquesEstimator::updateKinematicsFromFloatingBase(const JointPosDoubleArray& jointPos,
                                                                           const JointDOFsDoubleArray& jointVel,
                                                                           const JointDOFsDoubleArray& jointAcc,
                                                                           const FrameIndex& floatingFrame,
                                                                           const Vector3& properClassicalLinearAcceleration,
                                                                           const Vector3& angularVel,
                                                                           const Vector3& angularAcc)
{
    if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Model and sensors information not setted.");
        return false;
    }

    if( floatingFrame == FRAME_INVALID_INDEX ||
        floatingFrame < 0 || floatingFrame >= m_model.getNrOfFrames() )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Unknown frame index specified.");
        return false;
    }

    // Get link of the specified frame
    LinkIndex floatingLinkIndex = m_model.getFrameLink(floatingFrame);

    // Build the traversal if it is not present
    if( m_kinematicTraversals[floatingLinkIndex]->getNrOfVisitedLinks() == 0 )
    {
        m_model.computeFullTreeTraversal(*m_kinematicTraversals[floatingLinkIndex],floatingLinkIndex);
    }

    // To initialize the kinematic propagation, we should first convert the kinematics
    // information from the frame in which they are specified to the main frame of the link
    Transform link_H_frame = m_model.getFrameTransform(floatingFrame);

    Twist      base_vel_frame, base_vel_link;
    SpatialAcc base_acc_frame, base_acc_link;
    ClassicalAcc  base_classical_acc_link;

    Vector3 zero3;
    zero3.zero();

    base_vel_frame.setLinearVec3(zero3);
    base_vel_frame.setAngularVec3(angularVel);

    base_vel_link = link_H_frame*base_vel_frame;

    base_acc_frame.setLinearVec3(properClassicalLinearAcceleration);
    base_acc_frame.setAngularVec3(angularAcc);

    base_acc_link = link_H_frame*base_acc_frame;

    base_classical_acc_link.fromSpatial(base_acc_link,base_vel_link);

    // Propagate the kinematics information
    bool ok = dynamicsEstimationForwardVelAccKinematics(m_model,*(m_kinematicTraversals[floatingLinkIndex]),
                                                     base_classical_acc_link.getLinearVec3(),
                                                     base_vel_link.getAngularVec3(),
                                                     base_classical_acc_link.getAngularVec3(),
                                                     jointPos,jointVel,jointAcc,
                                                     m_linkVels,m_linkProperAccs);

    // Store joint positions
    m_jointPos = jointPos;

    if( !ok )
    {
        return false;
    }
    else
    {
        m_isKinematicsUpdated = true;
        return true;
    }
}

bool ExtWrenchesAndJointTorquesEstimator::computeExpectedFTSensorsMeasurements(const LinkUnknownWrenchContacts& unknowns,
                                                                                     SensorsMeasurements& predictedMeasures,
                                                                                     LinkContactWrenches& estimatedContactWrenches,
                                                                                     JointDOFsDoubleArray& estimatedJointTorques)
{
    if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","computeExpectedFTSensorsMeasurements",
                    "Model and sensors information not set.");
        return false;
    }

    if( !m_isKinematicsUpdated )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","computeExpectedFTSensorsMeasurements",
                    "Kinematic information not set.");
        return false;
    }

    /**
     * Compute external wrenches
     */
    bool ok = estimateExternalWrenchesWithoutInternalFT(m_model,m_dynamicTraversal,unknowns,
                                                        m_jointPos,m_linkVels,m_linkProperAccs,
                                                        m_calibBufs,estimatedContactWrenches);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in estimating the external contact wrenches without using the internal FT sensors.");
        return false;
    }

    /**
     * Compute net external wrenches
     */
    ok = ok && estimatedContactWrenches.computeNetWrenches(m_linkNetExternalWrenches);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in computing the net external wrenches from the estimated contact forces");
        return false;
    }

    /**
     * Compute joint torques
     */
    ok = ok && RNEADynamicPhase(m_model,m_dynamicTraversal,m_jointPos,m_linkVels,m_linkProperAccs,
                                m_linkNetExternalWrenches,m_linkIntWrenches,m_generalizedTorques);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in computing the dynamic phase of the RNEA.");
        return false;
    }

    /**
     * Simulate FT sensor measurements
     */
    predictSensorsMeasurementsFromRawBuffers(m_model,m_sensors,m_dynamicTraversal,
                                             m_linkVels,m_linkProperAccs,m_linkIntWrenches,predictedMeasures);


    /**
     * Copy the joint torques computed by the RNEA to the output
     */
    estimatedJointTorques = m_generalizedTorques.jointTorques();


    return ok;
}

bool ExtWrenchesAndJointTorquesEstimator::estimateExtWrenchesAndJointTorques(const LinkUnknownWrenchContacts& unknowns,
                                                                             const SensorsMeasurements& ftSensorsMeasures,
                                                                                   LinkContactWrenches& estimateContactWrenches,
                                                                                   JointDOFsDoubleArray& jointTorques)
{
    if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Model and sensors information not set.");
        return false;
    }

    if( !m_isKinematicsUpdated )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Kinematic information not set.");
        return false;
    }

    /**
     * Compute external forces
     */
    bool ok = estimateExternalWrenches(m_model,m_submodels,m_sensors,
                                       unknowns,m_jointPos,m_linkVels,m_linkProperAccs,
                                       ftSensorsMeasures,m_bufs,estimateContactWrenches);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in estimating the external contact wrenches");
        return false;
    }

    /**
     * Compute net external wrenches
     */
    ok = ok && estimateContactWrenches.computeNetWrenches(m_linkNetExternalWrenches);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in computing the net external wrenches from the estimated contact forces");
        return false;
    }

    /**
     * Compute joint torques
     */
    ok = ok && RNEADynamicPhase(m_model,m_dynamicTraversal,m_jointPos,m_linkVels,m_linkProperAccs,
                                m_linkNetExternalWrenches,m_linkIntWrenches,m_generalizedTorques);

    if( !ok )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateExtWrenchesAndJointTorques",
                    "Error in computing the dynamic phase of the RNEA.");
        return false;
    }

    /**
     * Copy the joint torques computed by the RNEA to the output
     */
    jointTorques = m_generalizedTorques.jointTorques();

    return ok;
}






}

