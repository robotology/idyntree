// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ExtWrenchesAndJointTorquesEstimator.h>
#include <iDynTree/ExternalWrenchesEstimation.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/EigenMathHelpers.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/ClassicalAcc.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/JointState.h>
#include <iDynTree/ContactWrench.h>
#include <iDynTree/Dynamics.h>
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/ModelSensorsTransformers.h>

#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>
#include <iDynTree/PredictSensorsMeasurements.h>

#include <iDynTree/ModelLoader.h>

#include <iDynTree/EigenHelpers.h>

#include <sstream>

namespace iDynTree
{

ExtWrenchesAndJointTorquesEstimator::ExtWrenchesAndJointTorquesEstimator():
    m_model(),
    m_submodels(),
    m_isModelValid(false),
    m_isKinematicsUpdated(false),
    m_dynamicTraversal(),
    m_kinematicTraversals(),
    m_jointPos(),
    m_jointVel(),
    m_jointAcc(),
    m_linkVels(),
    m_linkProperAccs(),
    m_linkNetExternalWrenches(),
    m_linkIntWrenches(),
    m_generalizedTorques(),
    m_calibBufs(),
    m_bufs()
{

}

ExtWrenchesAndJointTorquesEstimator::~ExtWrenchesAndJointTorquesEstimator()
{
}


void getFTJointNames(const SensorsList & _sensors,
                     std::vector<std::string>& ftJointNames)
{
    ftJointNames.resize(0);

    for(size_t sens=0; sens < _sensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE); sens++)
    {
        SixAxisForceTorqueSensor* pSens = static_cast<SixAxisForceTorqueSensor*>(_sensors.getSensor(SIX_AXIS_FORCE_TORQUE,sens));
        ftJointNames.push_back(pSens->getParentJoint());
    }

    return;
}


bool ExtWrenchesAndJointTorquesEstimator::setModelAndSensors(const Model& _model,
                                                             const SensorsList& _sensors)
{
    Model modelCopy = _model;
    modelCopy.sensors() = _sensors;

    return setModel(modelCopy);
}

bool ExtWrenchesAndJointTorquesEstimator::setModel(const Model& _model)
{
    m_model = _model;

    // resize the data structures
    m_model.computeFullTreeTraversal(m_dynamicTraversal);
    m_kinematicTraversals.resize(m_model);

    m_jointPos.resize(m_model);
    m_jointVel.resize(m_model);
    m_jointAcc.resize(m_model);
    m_linkVels.resize(m_model);
    m_linkProperAccs.resize(m_model);
    m_linkIntWrenches.resize(m_model);
    m_linkNetExternalWrenches.resize(m_model);
    m_generalizedTorques.resize(m_model);

    // create submodel structure
    std::vector<std::string> ftJointNames;
    getFTJointNames(m_model.sensors(),ftJointNames);
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
                                                                      const std::string filetype)
{
    ModelLoader loader;
    if (!loader.loadModelFromFile(filename, filetype)) {
        reportError("ExtWrenchesAndJointTorquesEstimator", "loadModelAndSensorsFromFile", "Error in parsing from URDF.");
        return false;
    }
    return setModelAndSensors(loader.model(), loader.sensors());
}

bool ExtWrenchesAndJointTorquesEstimator::loadModelAndSensorsFromFileWithSpecifiedDOFs(const std::string filename,
                                                                                       const std::vector< std::string >& consideredDOFs,
                                                                                       const std::string filetype)
{
    ModelLoader loader;
    if (!loader.loadModelFromFile(filename, filetype)) {
        reportError("ExtWrenchesAndJointTorquesEstimator", "loadModelAndSensorsFromFileWithSpecifiedDOFs", "Error in parsing from URDF.");
        return false;
    }

    Model _modelFull = loader.model();
    SensorsList _sensorsFull = loader.sensors();

    // We need to create a reduced model, inclusing only the consideredDOFs and the joints used by the FT sensors
    std::vector< std::string > consideredJoints = consideredDOFs;

    // Add FT joints (if they are not already in the consideredDOFs list
    std::vector< std::string > ftJointNames;
    getFTJointNames(_sensorsFull,ftJointNames);

    for (size_t i = 0; i < ftJointNames.size(); i++)
    {
        // Only add an F/T sensor joint if it is not already in consideredDOFs
        if (std::find(consideredJoints.begin(), consideredJoints.end(), ftJointNames[i]) == consideredJoints.end())
        {
            consideredJoints.push_back(ftJointNames[i]);
        }
    }


    Model _modelReduced;
    SensorsList _sensorsReduced;

    bool parsingCorrect = createReducedModelAndSensors(_modelFull,_sensorsFull,consideredJoints,_modelReduced,_sensorsReduced);

    if (!parsingCorrect)
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","loadModelAndSensorsFromFileWithSpecifiedDOFs","Error in creating reduced model and sensors.");
        return false;
    }

    return setModelAndSensors(_modelReduced,_sensorsReduced);
}


const Model& ExtWrenchesAndJointTorquesEstimator::model() const
{
    return m_model;
}

const SensorsList& ExtWrenchesAndJointTorquesEstimator::sensors() const
{
    return m_model.sensors();
}

const SubModelDecomposition& ExtWrenchesAndJointTorquesEstimator::submodels() const
{
    return m_submodels;
}


bool ExtWrenchesAndJointTorquesEstimator::updateKinematicsFromFixedBase(const VectorDynSize& jointPos,
                                                                        const VectorDynSize& jointVel,
                                                                        const VectorDynSize& jointAcc,
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


bool ExtWrenchesAndJointTorquesEstimator::updateKinematicsFromFloatingBase(const VectorDynSize& jointPos,
                                                                           const VectorDynSize& jointVel,
                                                                           const VectorDynSize& jointAcc,
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
        floatingFrame < 0 || floatingFrame >= static_cast<FrameIndex>(m_model.getNrOfFrames()) )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Unknown frame index specified.");
        return false;
    }

    if (m_jointPos.size() != jointPos.size())
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Wrong size of input joint positions.");
        return false;
    }

    if (m_jointVel.size() != jointVel.size())
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Wrong size of input joint velocities.");
    }

    if (m_jointAcc.size() != jointAcc.size())
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","updateKinematicsFromFloatingBase","Wrong size of input joint accelerations.");
    }

    m_jointPos = jointPos;
    m_jointVel = jointVel;
    m_jointAcc = jointAcc;

    // Get link of the specified frame
    LinkIndex floatingLinkIndex = m_model.getFrameLink(floatingFrame);

    // To initialize the kinematic propagation, we should first convert the kinematics
    // information from the frame in which they are specified to the main frame of the link
    Transform link_H_frame = m_model.getFrameTransform(floatingFrame);

    // Convert the twist from the additional  frame to the link frame
    Twist      base_vel_frame, base_vel_link;
    Vector3 zero3;
    zero3.zero();
    base_vel_frame.setLinearVec3(zero3);
    base_vel_frame.setAngularVec3(angularVel);
    base_vel_link = link_H_frame*base_vel_frame;

    // Convert the acceleration from the additional  frame to the link frame
    SpatialAcc base_acc_frame, base_acc_link;
    ClassicalAcc  base_classical_acc_link;
    base_acc_frame.setLinearVec3(properClassicalLinearAcceleration);
    base_acc_frame.setAngularVec3(angularAcc);
    base_acc_link = link_H_frame*base_acc_frame;
    base_classical_acc_link.fromSpatial(base_acc_link,base_vel_link);


    // Propagate the kinematics information
    bool ok = dynamicsEstimationForwardVelAccKinematics(m_model,m_kinematicTraversals.getTraversalWithLinkAsBase(m_model,floatingLinkIndex),
                                                        base_classical_acc_link.getLinearVec3(),
                                                        base_vel_link.getAngularVec3(),
                                                        base_classical_acc_link.getAngularVec3(),
                                                        m_jointPos,m_jointVel,m_jointAcc,
                                                        m_linkVels,m_linkProperAccs);


    m_isKinematicsUpdated = ok;
    return ok;
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
    predictSensorsMeasurementsFromRawBuffers(m_model,m_dynamicTraversal,
                                             m_linkVels,m_linkProperAccs,m_linkIntWrenches,predictedMeasures);


    /**
     * Copy the joint torques computed by the RNEA to the output
     */
    estimatedJointTorques = m_generalizedTorques.jointTorques();


    return ok;
}

Wrench computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics_ComputebHelper(const Model& model,
                                                              const Traversal& subModelTraversal,
                                                              const JointPosDoubleArray & jointPos,
                                                              const LinkVelArray& linkVel,
                                                              const LinkAccArray& linkProperAcc,
                                                                    estimateExternalWrenchesBuffers& bufs)
{
    // First compute the known term of the estimation for each link:
    // this loop is similar to the dynamic phase of the RNEA
    // \todo pimp up performance as done in RNEADynamicPhase
     for(int traversalEl = subModelTraversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
     {
         LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
         LinkIndex    visitedLinkIndex = visitedLink->getIndex();
         LinkConstPtr parentLink  = subModelTraversal.getParentLink(traversalEl);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAcc(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVel(visitedLinkIndex);
         bufs.b_contacts_subtree(visitedLinkIndex) = I*properAcc + v*(I*v);

         // Iterate on childs of visitedLink
         // We obtain all the children as all the neighbors of the link, except
         // for its parent
         // \todo TODO this point is definitly Tree-specific
         // \todo TODO this "get child" for is duplicated in the code, we
         //            should try to consolidate it
         for(unsigned int neigh_i=0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex); neigh_i++)
         {
             LinkIndex neighborIndex = model.getNeighbor(visitedLinkIndex,neigh_i).neighborLink;
             // Check if this neighbor is a child of the link according to this traversal
             if( subModelTraversal.isParentOf(visitedLinkIndex,neighborIndex) )
             {
                 LinkIndex childIndex = neighborIndex;
                 IJointConstPtr neighborJoint = model.getJoint(model.getNeighbor(visitedLinkIndex,neigh_i).neighborJoint);
                 Transform visitedLink_X_child = neighborJoint->getTransform(jointPos,visitedLinkIndex,childIndex);

                 // One term of the sum in Equation 5.20 in Featherstone 2008
                 bufs.b_contacts_subtree(visitedLinkIndex) = bufs.b_contacts_subtree(visitedLinkIndex)
                                                            + visitedLink_X_child*bufs.b_contacts_subtree(childIndex);
             }
         }

         if( parentLink == 0 )
         {
             // If the visited link is the base of the submodel, the
             // computed known terms is the known term of the submodel itself
             return bufs.b_contacts_subtree(visitedLinkIndex);
         }
     }

     // If we reach this point of the code, something is really really wrong
     assert(false);
     return Wrench::Zero();
}


// If it exists, get the link of the traversal to which the FT sensor is connected. If this link does not exists,
// return nullptr otherwise
// Warning: this function assumes that only one link in the traversal is connected to the FT sensors, so it is
// suitable to be used with submodels generated by some code similar to:
// std::vector<std::string> ftJointNames;
// getFTJointNames(m_model.sensors(),ftJointNames);
// bool ok = m_submodels.splitModelAlongJoints(m_model,m_dynamicTraversal,ftJointNames);
iDynTree::LinkConstPtr getLinkOfSubModelThatIsConnectedToFTSensors(const Traversal & subModelTraversal, iDynTree::SixAxisForceTorqueSensor * ftSens)
{
    for(size_t traversalEl = 0; traversalEl < subModelTraversal.getNrOfVisitedLinks(); traversalEl++)
    {
        iDynTree::LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);

        if (ftSens->isLinkAttachedToSensor(visitedLink->getIndex()))
        {
            return visitedLink;
        }
    }

    return nullptr;
}


bool ExtWrenchesAndJointTorquesEstimator::computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics(const LinkUnknownWrenchContacts & unknownWrenches,
                                                                                                      std::vector<iDynTree::MatrixDynSize>& A,
                                                                                                      std::vector<iDynTree::VectorDynSize>& b,
                                                                                                      std::vector<std::ptrdiff_t>& subModelIDs,
                                                                                                      std::vector<iDynTree::LinkIndex>& baseLinkIndeces)
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

    // Compute nrOfSubmodels without external forces, and specifically which submodel
    subModelIDs.resize(0);
    size_t nrOfSubModels = m_submodels.getNrOfSubModels();
    for(size_t sm=0; sm < nrOfSubModels; sm++)
    {
        bool subModelHasExternalWrenchOnIt = false;

        // Iterate over all links of the submodel, and check if there is any external contact
        const Traversal & subModelTraversal = m_submodels.getTraversal(sm);
        for(size_t traversalEl = 0; traversalEl < subModelTraversal.getNrOfVisitedLinks(); traversalEl++)
        {
            LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
            size_t nrOfContactForLink = unknownWrenches.getNrOfContactsForLink(visitedLink->getIndex());

            if (nrOfContactForLink > 0)
            {
                subModelHasExternalWrenchOnIt = true;
            }
        }

        if (!subModelHasExternalWrenchOnIt)
        {
            subModelIDs.push_back(sm);
        }
    }

    size_t nrOfSubModelsWithoutExtWrenches = subModelIDs.size();
    size_t nrOfFTSensors = m_model.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);

    // Resize quantities
    baseLinkIndeces.resize(nrOfSubModelsWithoutExtWrenches);
    A.resize(nrOfSubModelsWithoutExtWrenches);
    b.resize(nrOfSubModelsWithoutExtWrenches);

    for(size_t l=0; l < nrOfSubModelsWithoutExtWrenches; l++)
    {
        A[l].resize(6, 6*nrOfFTSensors);
        b[l].resize(6);
    }

    // Populate quantities
    for (size_t l = 0; l < subModelIDs.size(); l++)
    {
        size_t sm = subModelIDs[l];
        const Traversal & subModelTraversal = m_submodels.getTraversal(sm);

        // Assign baseLinkIndeces
        baseLinkIndeces[l] = subModelTraversal.getBaseLink()->getIndex();

        // Compute b vector

        // First compute the known term of the estimation for each link:
        // this loop is similar to the dynamic phase of the RNEA
        toEigen(b[l]) = toEigen(computeSubModelMatrixRelatingFTSensorsMeasuresAndKinematics_ComputebHelper(m_model,subModelTraversal,m_jointPos,m_linkVels,m_linkProperAccs,m_bufs));

        // Compute A matrix
        Eigen::Map< Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Aeig = toEigen(A[l]);
        Aeig.setZero();

        // As a first step, we need to compute the transform between each link and the base
        // of its submodel (we are computing the estimation equation in the submodel base frame
        computeTransformToTraversalBase(m_model,subModelTraversal,m_jointPos,m_bufs.subModelBase_H_link);

        // Iterate over all FTs of the full model, and check if they are connected to the submodel
        // TODO: if this O(n^2) loop turns out to be expensive, we can speed it up by caching somewhere
        // the set of FT sensors that belong to a submodel
        for(size_t ft=0; ft < nrOfFTSensors; ft++ )
        {
            ::iDynTree::SixAxisForceTorqueSensor * ftSens
                = (::iDynTree::SixAxisForceTorqueSensor *) m_model.sensors().getSensor(::iDynTree::SIX_AXIS_FORCE_TORQUE,ft);

            assert(ftSens != 0);

            iDynTree::LinkConstPtr linkConnectedToFt = getLinkOfSubModelThatIsConnectedToFTSensors(subModelTraversal, ftSens);

            if (linkConnectedToFt)
            {
                iDynTree::Matrix6x6 link_T_sens;
                bool ok = ftSens->getWrenchAppliedOnLinkMatrix(linkConnectedToFt->getIndex(), link_T_sens);
                IDYNTREE_UNUSED(ok);
                assert(ok);
                iDynTree::Transform subModelBase_T_link = m_bufs.subModelBase_H_link(linkConnectedToFt->getIndex());
                Aeig.block<6,6>(0, 6*ft) = iDynTree::toEigen(subModelBase_T_link.asAdjointTransformWrench())*iDynTree::toEigen(link_T_sens);
            }
        }

    }

    return true;
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
    bool ok = estimateExternalWrenches(m_model,m_submodels,m_model.sensors(),
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

bool ExtWrenchesAndJointTorquesEstimator::checkThatTheModelIsStill(const double gravityNorm,
                                                                   const double properAccTol,
                                                                   const double verbose)
{
    if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","checkThatTheModelIsStill",
                    "Model and sensors information not set.");
        return false;
    }

    if( !m_isKinematicsUpdated )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","checkThatTheModelIsStill",
                    "Kinematic information not set.");
        return false;
    }

    bool isStill = true;

    for(LinkIndex link = 0; link < static_cast<LinkIndex>(m_model.getNrOfLinks()); link++)
    {
        double properAccNorm = toEigen(m_linkProperAccs(link).getLinearVec3()).norm();

        if( fabs(properAccNorm-gravityNorm) >= properAccTol )
        {
            isStill = false;

            if( verbose )
            {
                std::ostringstream strs;
                strs << "Link " <<  this->m_model.getLinkName(link) << " has a proper acceleration of "
                     <<  m_linkProperAccs(link).getLinearVec3().toString() <<  " (norm : " <<  properAccNorm << ")";
                reportError("ExtWrenchesAndJointTorquesEstimator","checkThatTheModelIsStill",
                            strs.str().c_str());
            }
        }
    }

    return isStill;
}

bool ExtWrenchesAndJointTorquesEstimator::estimateLinkNetWrenchesWithoutGravity(LinkNetTotalWrenchesWithoutGravity& netWrenches)
{
   if( !m_isModelValid )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateLinkNetWrenchesWithoutGravity",
                    "Model and sensors information not set.");
        return false;
    }

    if( !m_isKinematicsUpdated )
    {
        reportError("ExtWrenchesAndJointTorquesEstimator","estimateLinkNetWrenchesWithoutGravity",
                    "Kinematic information not set.");
        return false;
    }

    netWrenches.resize(m_model);

    return computeLinkNetWrenchesWithoutGravity(m_model,m_linkVels,m_linkProperAccs,netWrenches);
}







}

