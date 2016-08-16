/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Estimation/BerdyHelper.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AllSensorsTypes.h>

#include <sstream>

namespace iDynTree
{

bool isJointBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch(dynamicVariableType)
    {
        case LINK_BODY_PROPER_ACCELERATION:
            return false;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return false;
        case JOINT_WRENCH:
            return true;
        case DOF_TORQUE:
            return false;
        case NET_EXT_WRENCH:
            return false;
        case DOF_ACCELERATION:
            return false;
        default:
            return false;
    }
}

bool isLinkBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch(dynamicVariableType)
    {
        case LINK_BODY_PROPER_ACCELERATION:
            return true;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return true;
        case JOINT_WRENCH:
            return false;
        case DOF_TORQUE:
            return false;
        case NET_EXT_WRENCH:
            return true;
        case DOF_ACCELERATION:
            return false;
        default:
            return false;
    }
}

bool isDOFBerdyDynamicVariable(const BerdyDynamicVariablesTypes dynamicVariableType)
{
    switch(dynamicVariableType)
    {
        case LINK_BODY_PROPER_ACCELERATION:
            return false;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            return false;
        case JOINT_WRENCH:
            return false;
        case DOF_TORQUE:
            return true;
        case NET_EXT_WRENCH:
            return false;
        case DOF_ACCELERATION:
            return true;
        default:
            return false;
    }
}

bool BerdyOptions::checkConsistency()
{
    if( this->includeAllNetExternalWrenchesAsSensors )
    {
        if( !this->includeAllNetExternalWrenchesAsDynamicVariables )
        {
            reportError("BerdyOptions","checkConsistency","Impossible to load berdy, as includeAllNetExternalWrenchesAsSensors is set to true but includeAllNetExternalWrenchesAsDynamicVariables is set to false");
            return false;
        }
    }

    return true;
}

bool BerdySensor::operator==(const struct BerdySensor &s)
{
    return s.type == this->type
    && s.id == this->id;
}

BerdyHelper::BerdyHelper(): m_areModelAndSensorsValid(false),
                              m_kinematicsUpdated(false),
                              m_nrOfDynamicalVariables(0),
                              m_nrOfDynamicEquations(0),
                              m_nrOfSensorsMeasurements(0)
{

}

bool BerdyHelper::init(const Model& model,
                        const SensorsList& sensors,
                        const BerdyOptions options)
{
    // Reset the class
    m_kinematicsUpdated = false;
    m_areModelAndSensorsValid = false;

    m_model = model;
    m_sensors = sensors;
    m_options = options;

    m_model.computeFullTreeTraversal(m_dynamicsTraversal);
    m_kinematicTraversals.resize(m_model);
    m_jointPos.resize(m_model);
    m_jointVel.resize(m_model);
    m_linkVels.resize(m_model);

    bool res = m_options.checkConsistency();

    if( !res )
    {
        reportError("BerdyHelpers","init","BerdyOptions not consistent.");
        return false;
    }

    switch(m_options.berdyVariant)
    {
        case ORIGINAL_BERDY_FIXED_BASE :
            res = initOriginalBerdyFixedBase();
            break;

        case BERDY_FLOATING_BASE :
            res = initBerdyFloatingBase();
            break;

        default:
            reportError("BerdyHelpers","init","unknown berdy variant");
            res = false;
    }


    if( res )
    {
        m_areModelAndSensorsValid = true;
    }
    else
    {
        reportError("BerdyHelpers","init","initialization failed");
    }

    return res;
}

BerdyOptions BerdyHelper::getOptions() const
{
    return m_options;
}


bool BerdyHelper::initSensorsMeasurements()
{
    // The number of sensors measurements is given by the sensors contained
    // in sensorsList (informally: the number of measurments of sensors
    // contained in the robot model) plus the additional/fictious sensors
    // specified in the BerdyOptions
    m_nrOfSensorsMeasurements = m_sensors.getSizeOfAllSensorsMeasurements();

    // The offset of joint accelerations
    berdySensorTypeOffsets.dofAccelerationOffset = m_nrOfSensorsMeasurements;

    if( m_options.includeAllJointAccelerationsAsSensors )
    {
        m_nrOfSensorsMeasurements += this->m_model.getNrOfDOFs();
    }

    // The offset of joint torques
    berdySensorTypeOffsets.dofTorquesOffset = m_nrOfSensorsMeasurements;

    if( m_options.includeAllJointTorquesAsSensors )
    {
        m_nrOfSensorsMeasurements += this->m_model.getNrOfDOFs();
    }

    berdySensorTypeOffsets.netExtWrenchOffset = m_nrOfSensorsMeasurements;
    if( m_options.includeAllNetExternalWrenchesAsSensors )
    {
        unsigned numOfExternalWrenches = this->m_model.getNrOfLinks();
        if (m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE
            && !m_options.includeFixedBaseExternalWrench)
            numOfExternalWrenches = this->m_model.getNrOfLinks() - 1;
        m_nrOfSensorsMeasurements += 6 * numOfExternalWrenches;
    }

    berdySensorTypeOffsets.jointWrenchOffset = m_nrOfSensorsMeasurements;

    // Check the considered joint wrenches are actually part of the model
    berdySensorsInfo.jntIdxToOffset.resize(m_model.getNrOfJoints(),JOINT_INVALID_INDEX);
    for(int i=0; i < m_options.jointOnWhichTheInternalWrenchIsMeasured.size(); i++)
    {
        JointIndex jntIdx = m_model.getJointIndex(m_options.jointOnWhichTheInternalWrenchIsMeasured[i]);

        if( jntIdx == JOINT_INVALID_INDEX )
        {
            std::stringstream ss;
            ss << "unknown joint " << m_options.jointOnWhichTheInternalWrenchIsMeasured[i];
            reportError("BerdyHelper","initSensorsMeasurements",ss.str().c_str());
            return false;
        }

        berdySensorsInfo.wrenchSensors.push_back(jntIdx);
        berdySensorsInfo.jntIdxToOffset[jntIdx] = m_nrOfSensorsMeasurements;
        m_nrOfSensorsMeasurements += 6;
    }

    return true;
}


bool BerdyHelper::initOriginalBerdyFixedBase()
{
    // Check that all joints have 1 dof
    for(JointIndex jntIdx = 0; jntIdx < m_model.getNrOfJoints(); jntIdx++)
    {
        if( m_model.getJoint(jntIdx)->getNrOfDOFs() != 1 )
        {
            std::stringstream ss;
            ss << "Joint " << m_model.getJointName(jntIdx) << " has " << m_model.getJoint(jntIdx)->getNrOfDOFs() << " DOFs , but the original fixed base formulation of berdy only supports 1 dof frames";
            reportError("BerdyHelpers","init",ss.str().c_str());
            return false;
        }
    }

    // Check that no sensors are attached to the base (this is not supported by the original berdy)
    for(SensorType type=SIX_AXIS_FORCE_TORQUE; type < NR_OF_SENSOR_TYPES; type = (SensorType)(type+1))
    {
        // TODO : link sensor are extremly widespared and they have all approximatly the same API,
        //        so we need a better way to iterate over them
        if( isLinkSensor(type) )
        {
            for(size_t sensIdx = 0; sensIdx < m_sensors.getNrOfSensors(type); sensIdx++)
            {
                LinkSensor * linkSensor = dynamic_cast<LinkSensor*>(m_sensors.getSensor(type,sensIdx));

                LinkIndex   linkToWhichTheSensorIsAttached = linkSensor->getParentLinkIndex();

                if( linkToWhichTheSensorIsAttached == m_dynamicsTraversal.getBaseLink()->getIndex() )
                {
                    std::stringstream ss;
                    ss << "Sensor " << linkSensor->getName() << " is attached to link " << m_model.getLinkName(linkToWhichTheSensorIsAttached) << " but this link is the base link and base link sensors are not supported by the original berdy.";
                    reportError("BerdyHelpers","init",ss.str().c_str());
                    return false;
                }
            }
        }
    }


    // N will be the number of joints/links/dofs
    // We assume that the fixed base is fixed, so we include in the model only N links
    size_t nrOfDOFs = m_model.getNrOfDOFs();

    // In the classical Berdy formulation, we have 6*4 + 2*1 dynamical variables
    size_t nrOfDynamicVariablesForDOFs = m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 6*4+2*1 : 6*3+2*1;
    m_nrOfDynamicalVariables = nrOfDynamicVariablesForDOFs*nrOfDOFs;
    // 19 Dynamics equations are considered in the original berdy :
    // 6 for the propagation of 6d acceleration
    // 6 for the computation of the net wrench from acc and vel
    // 6 for the computation of the joint wrench
    // 1 for the torque computations
    m_nrOfDynamicEquations   = (18+1)*nrOfDOFs;

    initSensorsMeasurements();

    return true;
}

IndexRange BerdyHelper::getRangeOriginalBerdyFixedBase(BerdyDynamicVariablesTypes dynamicVariableType, TraversalIndex idx)
{
    IndexRange ret;
    size_t dynamicVariableSize, dynamicVariableOffset, nrOfVariablesForJoint;

    if( m_options.includeAllNetExternalWrenchesAsDynamicVariables )
    {
        nrOfVariablesForJoint = 26;
    }
    else
    {
        nrOfVariablesForJoint = 20;
    }

    switch(dynamicVariableType)
    {
        case LINK_BODY_PROPER_ACCELERATION:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 0;
            break;
        case NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 6;
            break;
        case JOINT_WRENCH:
            dynamicVariableSize = 6;
            dynamicVariableOffset = 12;
            break;
        case DOF_TORQUE:
            dynamicVariableSize = 1;
            dynamicVariableOffset = 18;
            break;
        case NET_EXT_WRENCH:
            dynamicVariableSize = m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 6 : 0;
            dynamicVariableOffset = 19;
            break;
        case DOF_ACCELERATION:
            dynamicVariableSize = 1;
            dynamicVariableOffset = m_options.includeAllNetExternalWrenchesAsDynamicVariables ? 25 : 19;
            break;
    }

    // Note that the traversal index 0 is for the base link, that is not considered
    // in the fixed base Berdy, so we start from the link with traversal index 1
    ret.offset = nrOfVariablesForJoint*(idx-1)+dynamicVariableOffset;
    ret.size = dynamicVariableSize;

    return ret;
}

IndexRange BerdyHelper::getRangeLinkVariable(BerdyDynamicVariablesTypes dynamicVariableType, LinkIndex idx)
{
    assert(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if( !isLinkBerdyDynamicVariable(dynamicVariableType) )
    {
        return IndexRange::InvalidRange();
    }

    return getRangeOriginalBerdyFixedBase(dynamicVariableType,m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx));
}

// \todo TODO remove
TraversalIndex getTraversalIndexFromJointIndex(const Model & m_model, const Traversal & m_traversal, const JointIndex idx)
{
    LinkIndex childLink = m_traversal.getChildLinkIndexFromJointIndex(m_model,idx);

    return m_traversal.getTraversalIndexFromLinkIndex(childLink);
}


IndexRange BerdyHelper::getRangeJointVariable(BerdyDynamicVariablesTypes dynamicVariableType, JointIndex idx)
{
    assert(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if( !isJointBerdyDynamicVariable(dynamicVariableType) )
    {
        assert(false);
        return IndexRange::InvalidRange();
    }

    return getRangeOriginalBerdyFixedBase(dynamicVariableType,getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,idx));
}

IndexRange BerdyHelper::getRangeDOFVariable(BerdyDynamicVariablesTypes dynamicVariableType, DOFIndex idx)
{
    assert(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if( !isDOFBerdyDynamicVariable(dynamicVariableType) )
    {
        return IndexRange::InvalidRange();
    }

    // For a model with all 1-dofs, dof index and joint index matches
    return getRangeOriginalBerdyFixedBase(dynamicVariableType,getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,idx));
}

IndexRange BerdyHelper::getRangeSensorVariable(const SensorType type, const unsigned int sensorIdx)
{
    unsigned int sensorOffset = 0;

    if( type > SIX_AXIS_FORCE_TORQUE )
    {
        sensorOffset += 6*m_sensors.getNrOfSensors(SIX_AXIS_FORCE_TORQUE);
    }

    if( type > ACCELEROMETER )
    {
        sensorOffset += 3*m_sensors.getNrOfSensors(ACCELEROMETER);
    }

    if( type > GYROSCOPE )
    {
        sensorOffset += 3*m_sensors.getNrOfSensors(GYROSCOPE);
    }

    sensorOffset += sensorIdx*getSensorTypeSize(type);

    IndexRange ret;
    ret.offset = sensorOffset;
    ret.size = getSensorTypeSize(type);

    return ret;
}

IndexRange BerdyHelper::getRangeDOFSensorVariable(const BerdySensorTypes sensorType, const DOFIndex idx)
{
    IndexRange ret = IndexRange::InvalidRange();
    ret.size = 1;

    if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        // For ORIGINAL_BERDY_FIXED_BASE we know  that DOFIndex is always equalt to JointIndex
        TraversalIndex trvIdx = getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,(JointIndex)idx);

        if( sensorType == DOF_ACCELERATION_SENSOR )
        {
            ret.offset = berdySensorTypeOffsets.dofAccelerationOffset + (trvIdx-1);
        }

        if( sensorType == DOF_TORQUE_SENSOR )
        {
            ret.offset = berdySensorTypeOffsets.dofTorquesOffset + (trvIdx-1);
        }
    }

    return ret;
}

IndexRange BerdyHelper::getRangeJointSensorVariable(const BerdySensorTypes sensorType, const JointIndex idx)
{
    IndexRange ret = IndexRange::InvalidRange();
    ret.size = 6;

    if( sensorType == JOINT_WRENCH_SENSOR )
    {
        if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
        {
            ret.offset = berdySensorsInfo.jntIdxToOffset[idx];
        }
    }

    return ret;
}

IndexRange BerdyHelper::getRangeLinkSensorVariable(const BerdySensorTypes sensorType, const LinkIndex idx)
{
    IndexRange ret = IndexRange::InvalidRange();
    ret.size = 6;

    if( sensorType == NET_EXT_WRENCH_SENSOR )
    {
        if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
        {
            TraversalIndex trvIdx = m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);

            if( m_options.includeFixedBaseExternalWrench )
            {
                ret.offset = berdySensorTypeOffsets.netExtWrenchOffset + 6*trvIdx;
            }
            else
            {
                ret.offset = berdySensorTypeOffsets.netExtWrenchOffset + 6*(trvIdx-1);
            }
        }
    }

    assert(ret.isValid());
    return ret;
}





IndexRange BerdyHelper::getRangeLinkProperAccDynEq(const LinkIndex idx)
{
    IndexRange ret;

    if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        TraversalIndex trvIdx = m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);
        ret.offset = 19*(trvIdx-1)+0;
        ret.size = 6;
        return ret;
    }

    return IndexRange::InvalidRange();
}

IndexRange BerdyHelper::getRangeLinkNetTotalwrenchDynEq(const LinkIndex idx)
{
    IndexRange ret;

    if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        TraversalIndex trvIdx = m_dynamicsTraversal.getTraversalIndexFromLinkIndex(idx);
        ret.offset = 19*(trvIdx-1)+6;
        ret.size = 6;
        return ret;
    }

    return IndexRange::InvalidRange();
}

IndexRange BerdyHelper::getRangeJointWrench(const JointIndex idx)
{
    IndexRange ret;

    if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        TraversalIndex trvIdx = getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,idx);
        ret.offset = 19*(trvIdx-1)+12;
        ret.size = 6;
        return ret;
    }

    return IndexRange::InvalidRange();
}

IndexRange BerdyHelper::getRangeDOFTorqueDynEq(const DOFIndex idx)
{
    IndexRange ret;

    if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        TraversalIndex trvIdx = getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,idx);
        ret.offset = 19*(trvIdx-1)+18;
        ret.size = 1;
        return ret;
    }

    return IndexRange::InvalidRange();
}


bool BerdyHelper::computeBerdyDynamicsMatrices(MatrixDynSize& D, VectorDynSize& bD)
{
    D.resize(m_nrOfDynamicEquations,m_nrOfDynamicalVariables);
    bD.resize(m_nrOfDynamicEquations);
    // \todo TODO check if this is a bottleneck
    D.zero();
    bD.zero();

    // We follow the traversal skipping the base because all the equations of the RNEA are not directly affecting the base
    for(TraversalIndex traversalEl = 1; traversalEl < m_dynamicsTraversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = m_dynamicsTraversal.getLink(traversalEl);
        LinkConstPtr parentLink  = m_dynamicsTraversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = m_dynamicsTraversal.getParentJoint(traversalEl);
        LinkIndex    visitedLinkIdx = visitedLink->getIndex();
        LinkIndex    parentLinkIdx   = parentLink->getIndex();

        ///////////////////////////////////////////////////////////
        // Acceleration forward kinematics propagation (D part)
        ///////////////////////////////////////////////////////////
        const Transform & parent_X_visited = toParentJoint->getTransform(m_jointPos,parentLinkIdx,visitedLinkIdx);
        const Transform & visited_X_parent = toParentJoint->getTransform(m_jointPos,visitedLinkIdx,parentLinkIdx);

        // Proper acc propagation equations
        setSubMatrixToMinusIdentity(D,
                                    getRangeLinkProperAccDynEq(visitedLinkIdx),
                                    getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,visitedLinkIdx));

        // This should not be added if the variant is ORIGINAL_BERDY_FIXED_BASE and the parentLink is the base
        // because in that case the acceleration of the base is not a dynamic variable
        if( !(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex()) )
        {
            setSubMatrix(D,
                           getRangeLinkProperAccDynEq(visitedLinkIdx),
                           getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,parentLinkIdx),
                           visited_X_parent.asAdjointTransform());
        }

        size_t jointDOFs = toParentJoint->getNrOfDOFs();
        size_t dofOffset = toParentJoint->getDOFsOffset();

        for(size_t localDof = 0; localDof < jointDOFs; localDof++)
        {
            SpatialMotionVector S = toParentJoint->getMotionSubspaceVector(localDof,visitedLinkIdx,parentLinkIdx);
            setSubMatrix(D,
                         getRangeLinkProperAccDynEq(visitedLinkIdx),
                         getRangeDOFVariable(DOF_ACCELERATION,dofOffset+localDof),
                         toEigen(S));
        }

        ///////////////////////////////////////////////////////////
        // Acceleration forward kinematics propagation (bD part)
        ///////////////////////////////////////////////////////////
        SpatialMotionVector biasAcc;
        biasAcc.zero();

        for(size_t localDof = 0; localDof < jointDOFs; localDof++)
        {
            SpatialMotionVector S = toParentJoint->getMotionSubspaceVector(localDof,visitedLinkIdx,parentLinkIdx);
            biasAcc = biasAcc + m_linkVels(visitedLinkIdx).cross(S*m_jointVel(dofOffset+localDof));
        }

        if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex() )
        {
            // If the variant is ORIGINAL_BERDY_FIXED_BASE variant, we need to account explicitly for the gravity for links whose parent is the base
            biasAcc = biasAcc - visited_X_parent*m_gravity6D;
        }

        setSubVector(bD,
                     getRangeLinkProperAccDynEq(visitedLinkIdx),
                     toEigen(biasAcc));


        if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
        {
            ///////////////////////////////////////////////////////////
            // Total net wrench without gravity (D part, only for ORIGINAL_BERDY_FIXED_BASE
            ///////////////////////////////////////////////////////////
             setSubMatrixToMinusIdentity(D,
                                         getRangeLinkNetTotalwrenchDynEq(visitedLinkIdx),
                                         getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,visitedLinkIdx));

             setSubMatrix(D,
                          getRangeLinkNetTotalwrenchDynEq(visitedLinkIdx),
                          getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,visitedLinkIdx),
                          visitedLink->getInertia().asMatrix());

            ///////////////////////////////////////////////////////////
            // Total net wrench without gravity (bD part, only for ORIGINAL_BERDY_FIXED_BASE
            ///////////////////////////////////////////////////////////
            setSubVector(bD,
                         getRangeLinkNetTotalwrenchDynEq(visitedLinkIdx),
                         toEigen(m_linkVels(visitedLinkIdx)*(visitedLink->getInertia()*m_linkVels(visitedLinkIdx))));

        }

        ///////////////////////////////////////////////////////////
        // Joint wrench (D part)
        ///////////////////////////////////////////////////////////

        // Joint wrench itself
        setSubMatrixToMinusIdentity(D,
                                    getRangeJointWrench(toParentJoint->getIndex()),
                                    getRangeJointVariable(JOINT_WRENCH,toParentJoint->getIndex()));

        // Wrench of child links
        // Iterate on childs of visitedLink
        // We obtain all the children as all the neighbors of the link, except
        // for its parent
        // \todo TODO this point is definitly Tree-specific
        // \todo TODO this "get child" for is duplicated in the code, we
        //            should try to consolidate it
        for(unsigned int neigh_i=0; neigh_i < m_model.getNrOfNeighbors(visitedLinkIdx); neigh_i++)
        {
             LinkIndex neighborIndex = m_model.getNeighbor(visitedLinkIdx,neigh_i).neighborLink;
             if( !parentLink || neighborIndex != parentLink->getIndex() )
             {
                 LinkIndex childIndex = neighborIndex;
                 IJointConstPtr neighborJoint = m_model.getJoint(m_model.getNeighbor(visitedLinkIdx,neigh_i).neighborJoint);
                 const Transform & visitedLink_X_child = neighborJoint->getTransform(m_jointPos,visitedLinkIdx,childIndex);

                 setSubMatrix(D,
                              getRangeJointWrench(toParentJoint->getIndex()),
                              getRangeJointVariable(JOINT_WRENCH,neighborJoint->getIndex()),
                              visitedLink_X_child.asAdjointTransformWrench());

             }
        }

        // Net external wrench
        if( m_options.includeAllNetExternalWrenchesAsDynamicVariables )
        {
            setSubMatrixToMinusIdentity(D,
                                        getRangeJointWrench(toParentJoint->getIndex()),
                                        getRangeLinkVariable(NET_EXT_WRENCH,visitedLinkIdx));
        }


        if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
        {
            // In the ORIGINAL_BERDY_FIXED_BASE, we also add to the rows of the joint wrenches
            // the depend on the total wrenches
            setSubMatrixToIdentity(D,
                                   getRangeJointWrench(toParentJoint->getIndex()),
                                   getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,visitedLinkIdx));
        }

        if( m_options.berdyVariant == BERDY_FLOATING_BASE )
        {
            // In the floating base variant, we embed all the inertia related terms directly in the floaging base
             setSubMatrix(D,
                          getRangeJointWrench(toParentJoint->getIndex()),
                          getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,visitedLinkIdx),
                          visitedLink->getInertia().asMatrix());
        }

        ///////////////////////////////////////////////////////////
        // Joint wrench (bD part, only for BERDY_FLOATING_BASE)
        ///////////////////////////////////////////////////////////
        if( m_options.berdyVariant == BERDY_FLOATING_BASE )
        {
            setSubVector(bD,
                         getRangeJointWrench(toParentJoint->getIndex()),
                         toEigen(m_linkVels(visitedLinkIdx).cross(visitedLink->getInertia()*m_linkVels(visitedLinkIdx))));
        }

        ////////////////////////////////////////////////////////////
        /// DOF torques equations (D part, bD is always zero)
        ////////////////////////////////////////////////////////////
        for(size_t localDof = 0; localDof < jointDOFs; localDof++)
        {
            SpatialMotionVector S = toParentJoint->getMotionSubspaceVector(localDof,visitedLinkIdx,parentLinkIdx);
            setSubMatrix(D,
                         getRangeDOFTorqueDynEq(dofOffset+localDof),
                         getRangeJointVariable(JOINT_WRENCH,toParentJoint->getIndex()),
                         toEigen(S).transpose());
            setSubMatrix(D,
                         getRangeDOFTorqueDynEq(dofOffset+localDof),
                         getRangeDOFVariable(DOF_TORQUE,dofOffset+localDof),
                         -1.0);
        }
    }

    return true;
}

bool BerdyHelper::computeBerdySensorMatrices(MatrixDynSize& Y, VectorDynSize& bY)
{
    Y.resize(m_nrOfSensorsMeasurements,m_nrOfDynamicalVariables);
    bY.resize(m_nrOfSensorsMeasurements);
    // \todo TODO check if this is a bottleneck
    Y.zero();
    bY.zero();


    // For now handle all sensor types explicitly TODO clean this up

    ////////////////////////////////////////////////////////////////////////
    ///// SIX AXIS F/T SENSORS
    ////////////////////////////////////////////////////////////////////////
    size_t numOfFTs = m_sensors.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    for(size_t idx = 0; idx<numOfFTs; idx++)
    {
        SixAxisForceTorqueSensor * ftSens = (SixAxisForceTorqueSensor*)m_sensors.getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE, idx);
        LinkIndex childLink;
        childLink = m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model,ftSens->getParentJointIndex());
        Matrix6x6 sensor_M_link;
        ftSens->getWrenchAppliedOnLinkInverseMatrix(childLink,sensor_M_link);
        IndexRange sensorRange = this->getRangeSensorVariable(SIX_AXIS_FORCE_TORQUE,idx);
        IndexRange jointWrenchRange = this->getRangeJointVariable(JOINT_WRENCH,ftSens->getParentJointIndex());
        setSubMatrix(Y,sensorRange,jointWrenchRange,sensor_M_link);

        // bY for the F/T sensor is equal to zero
    }

    ////////////////////////////////////////////////////////////////////////
    ///// ACCELEROMETERS
    ////////////////////////////////////////////////////////////////////////
    unsigned int numAccl = m_sensors.getNrOfSensors(iDynTree::ACCELEROMETER);
    for(size_t idx = 0; idx<numAccl; idx++)
    {
        AccelerometerSensor * accelerometer = (AccelerometerSensor *)m_sensors.getSensor(iDynTree::ACCELEROMETER, idx);
        LinkIndex parentLinkId = accelerometer->getParentLinkIndex();
        Transform sensor_X_link = accelerometer->getLinkSensorTransform().inverse();
        IndexRange sensorRange = this->getRangeSensorVariable(ACCELEROMETER,idx);
        IndexRange linkBodyProperAcRange = this->getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,parentLinkId);

        // Y(sensorRange,linkBodyProperAcRange) for the accelerometer is the first three rows of the sensor_X_link adjoint matrix
        setSubMatrix(Y,sensorRange,linkBodyProperAcRange,toEigen(sensor_X_link.asAdjointTransform()).block<3,6>(0,0));

        Twist vSensor = sensor_X_link*m_linkVels(parentLinkId);

        // bY for the accelerometer is the cross product of the angular part and the linear part of the body 6D velocity expressed in the sensor orientation and wrt to the
        setSubVector(bY,sensorRange, toEigen(vSensor.getAngularVec3().cross(vSensor.getLinearVec3())) );
    }

    ////////////////////////////////////////////////////////////////////////
    ///// GYROSCOPES
    ////////////////////////////////////////////////////////////////////////
    unsigned int numGyro = m_sensors.getNrOfSensors(iDynTree::GYROSCOPE);
    for(size_t idx = 0; idx<numGyro; idx++)
    {
        GyroscopeSensor * gyroscope = (GyroscopeSensor*)m_sensors.getSensor(iDynTree::GYROSCOPE, idx);
        LinkIndex parentLinkId = gyroscope->getParentLinkIndex();
        Transform sensor_X_link = gyroscope->getLinkSensorTransform().inverse();
        IndexRange sensorRange = this->getRangeSensorVariable(GYROSCOPE,idx);

        // Y for the gyroscope is already zero

        // bY for the gyroscope is just the angular velocity of the link rotated in the sensor frame
        setSubVector(bY,sensorRange,toEigen((sensor_X_link*m_linkVels(parentLinkId)).getAngularVec3()));
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT ACCELERATIONS
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllJointAccelerationsAsSensors )
    {
        for(DOFIndex idx = 0; idx< this->m_model.getNrOfDOFs(); idx++)
        {
            // Y for the joint accelerations is just a rows of 0s and one 1  corresponding to the location
            // of the relative joint acceleration in the dynamic variables vector
            IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR,idx);
            IndexRange jointAccRange = this->getRangeDOFVariable(DOF_ACCELERATION,idx);

            setSubMatrix(Y,sensorRange,jointAccRange,1.0);

            // bY for the joint acceleration is zero
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT TORQUES
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllJointTorquesAsSensors )
    {
        for(DOFIndex idx = 0; idx< this->m_model.getNrOfDOFs(); idx++)
        {
            // Y for the joint torques is just a rows of 0s and one 1  corresponding to the location
            // of the relative joint torques in the dynamic variables vector
            IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR,idx);
            IndexRange jointTrqRange = this->getRangeDOFVariable(DOF_TORQUE,idx);

            setSubMatrix(Y,sensorRange,jointTrqRange,1.0);

            // bY for the joint torques is zero
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// NET EXTERNAL WRENCHES ACTING ON LINKS
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllNetExternalWrenchesAsSensors )
    {
        for(LinkIndex idx = 0; idx < this->m_model.getNrOfLinks(); idx++)
        {
            // If this link is the (fixed) base link and the
            // berdy variant is ORIGINAL_BERDY_FIXED_BASE , then
            // the net wrench applied on the base is not part of the dynamical
            // system. Anyhow, we can still write the base wrench as a function
            // of sum of the joint wrenches of all the joints attached to the base (tipically just one)
            if( m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE &&
                m_dynamicsTraversal.getBaseLink()->getIndex() == idx )
            {
                if(  m_options.includeFixedBaseExternalWrench  )
                {
                    IndexRange sensorRange = this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR,idx);
                    // Y encodes the (time varyng) relation between the wrenches transmitted by the joint attached
                    // to the base and net external wrench applied on the robot
                    // \todo TODO this "get child" for is duplicated in the code, we
                    // should try to consolidate it
                    for(unsigned int neigh_i=0; neigh_i < m_model.getNrOfNeighbors(idx); neigh_i++)
                    {
                        LinkIndex neighborIndex = m_model.getNeighbor(idx,neigh_i).neighborLink;
                        LinkIndex childIndex = neighborIndex;
                        IJointConstPtr neighborJoint = m_model.getJoint(m_model.getNeighbor(idx,neigh_i).neighborJoint);
                        const Transform & base_X_child = neighborJoint->getTransform(m_jointPos,idx,childIndex);

                        setSubMatrix(Y,
                                    getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR,idx),
                                    getRangeJointVariable(JOINT_WRENCH,neighborJoint->getIndex()),
                                    toEigen(base_X_child.asAdjointTransformWrench()));
                    }

                    // bY encodes the weight of the base link due to gravity (we omit the v*I*v as it is always zero)
                    Wrench baseLinkNetTotalWrenchesWithoutGrav = -(m_model.getLink(idx)->getInertia()*m_gravity6D);
                    setSubVector(bY,getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR,idx),toEigen(baseLinkNetTotalWrenchesWithoutGrav));
                }
            }
            else
            {
                // Y for the net external wrenches is just
                // six rows of 0 with an identity placed at the location
                // of the net external wrenches in the dynamic variable vector
                IndexRange sensorRange = this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR,idx);
                IndexRange netExtWrenchRange = this->getRangeLinkVariable(NET_EXT_WRENCH,idx);

                setSubMatrixToIdentity(Y,sensorRange,netExtWrenchRange);

                // bY for the net external wrenches is zero
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT WRENCHES
    ////////////////////////////////////////////////////////////////////////
    for(size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++)
    {
        // Y for the joint wrenches is just
        // six rows of 0 with an identity placed at the location
        // of the joint wrenches in the dynamic variable vector
        IndexRange sensorRange = this->getRangeJointSensorVariable(JOINT_WRENCH_SENSOR,berdySensorsInfo.wrenchSensors[i]);
        IndexRange jointWrenchOffset = this->getRangeJointVariable(JOINT_WRENCH,berdySensorsInfo.wrenchSensors[i]);

        assert(sensorRange.size == 6);
        assert(jointWrenchOffset.size == 6);
        setSubMatrixToIdentity(Y,sensorRange,jointWrenchOffset);

        // bY for the joint wrenches is zero
    }

    return true;
}


bool BerdyHelper::initBerdyFloatingBase()
{
    bool res = false;

    assert(false);
    //

    initSensorsMeasurements();

    return res;
}

Model& BerdyHelper::model()
{
    return this->m_model;
}

const Model& BerdyHelper::model() const
{
    return this->m_model;
}

SensorsList& BerdyHelper::sensors()
{
    return this->m_sensors;
}

const SensorsList& BerdyHelper::sensors() const
{
    return this->m_sensors;
}

const Traversal& BerdyHelper::dynamicTraversal()
{
    return this->m_dynamicsTraversal;
}



size_t BerdyHelper::getNrOfDynamicVariables() const
{
    return m_nrOfDynamicalVariables;
}

size_t BerdyHelper::getNrOfDynamicEquations() const
{
    return m_nrOfDynamicEquations;
}

size_t BerdyHelper::getNrOfSensorsMeasurements() const
{
    return m_nrOfSensorsMeasurements;
}

bool BerdyHelper::resizeAndZeroBerdyMatrices(MatrixDynSize& D, VectorDynSize& bD,
                                            MatrixDynSize& Y, VectorDynSize& bY)
{
    D.resize(getNrOfDynamicEquations(),getNrOfDynamicVariables());
    bD.resize(getNrOfDynamicEquations());
    Y.resize(getNrOfSensorsMeasurements(),getNrOfDynamicVariables());
    bY.resize(getNrOfSensorsMeasurements());
    D.zero();
    bD.zero();
    Y.zero();
    bY.zero();
    return true;
}

bool BerdyHelper::updateKinematicsFromTraversalFixedBase(const JointPosDoubleArray& jointPos,
                                                         const JointDOFsDoubleArray& jointVel,
                                                         const Vector3& gravity)
{
    return updateKinematicsFromFixedBase(jointPos,jointVel,m_dynamicsTraversal.getBaseLink()->getIndex(),gravity);
}


bool BerdyHelper::updateKinematicsFromFixedBase(const JointPosDoubleArray& jointPos,
                                                 const JointDOFsDoubleArray& jointVel,
                                                 const FrameIndex& fixedFrame,
                                                 const Vector3& gravity)
{
    m_gravity = gravity;
    m_gravity6D.setLinearVec3(gravity);
    AngularMotionVector3 zero3;
    zero3.zero();
    m_gravity6D.setAngularVec3(zero3);

    Vector3 zeroAngularVel;
    zeroAngularVel.zero();

    return updateKinematicsFromFloatingBase(jointPos,jointVel,fixedFrame,zeroAngularVel);
}

bool BerdyHelper::updateKinematicsFromFloatingBase(const JointPosDoubleArray& jointPos,
                                                    const JointDOFsDoubleArray& jointVel,
                                                    const FrameIndex& floatingFrame,
                                                    const Vector3& angularVel)
{
    if( !m_areModelAndSensorsValid )
    {
        reportError("BerdyHelpers","updateKinematicsFromFloatingBase","Model and sensors information not setted.");
        return false;
    }

    if( floatingFrame == FRAME_INVALID_INDEX ||
        floatingFrame < 0 || floatingFrame >= m_model.getNrOfFrames() )
    {
        reportError("BerdyHelpers","updateKinematicsFromFloatingBase","Unknown frame index specified.");
        return false;
    }

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

    // Propagate the kinematics information
    bool ok = dynamicsEstimationForwardVelKinematics(m_model,m_kinematicTraversals.getTraversalWithLinkAsBase(m_model,floatingLinkIndex),
                                                        base_vel_link.getAngularVec3(),
                                                        jointPos,jointVel,
                                                        m_linkVels);

    // The jointPos and joint vel are stored directly, as are then passed to the Model object to get adjacent links transforms
    m_jointPos = jointPos;
    m_jointVel = jointVel;

    m_kinematicsUpdated = ok;
    return ok;
}

bool BerdyHelper::getBerdyMatrices(MatrixDynSize& D, VectorDynSize& bD,
                                    MatrixDynSize& Y, VectorDynSize& bY)
{
    if( !m_kinematicsUpdated )
    {
        reportError("BerdyHelpers","getBerdyMatrices",
                    "Kinematic information not set.");
        return false;
    }


    bool res = true;

    bool ok;
    // Compute D matrix of dynamics equations
    ok = computeBerdyDynamicsMatrices(D,bD);

    // Compute Y matrix of sensors
    ok = computeBerdySensorMatrices(Y,bY);

    res = res && ok;

    return res;
}

    std::vector<BerdySensor> BerdyHelper::getSensorsOrdering()
    {
        //???: we can probably cache the result of this function in the init
        //TODO: reserve space
        unsigned size = 0;
        std::vector<BerdySensor> sensorOrdering;
        sensorOrdering.reserve(size);

        //To be a bit more flexible, rely on getRangeSensorVariable to have the order of
        //the URDF sensors
        //???: Isn't this a loop in some way??
        for (SensorsList::Iterator it = m_sensors.allSensorsIterator();
             it.isValid(); ++it)
        {
            IndexRange sensorRange = this->getRangeSensorVariable((*it)->getSensorType(), m_sensors.getSensorIndex((*it)->getSensorType(), (*it)->getName()));

            BerdySensor sensor;
            sensor.type = static_cast<BerdySensorTypes>((*it)->getSensorType());
            sensor.id = (*it)->getName();
            sensor.range = sensorRange;
            sensorOrdering.push_back(sensor);
        }


        //For each URDF sensor, get the offset with the function
        //put everything in a sorted list (sort with the offset)
        //the iterate and add the sensors to the vector

        //The remaining sensor order is hardcoded for now
        if (m_options.includeAllJointAccelerationsAsSensors)
        {
            for (DOFIndex idx = 0; idx < this->m_model.getNrOfDOFs(); idx++)
            {
                IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR,idx);
                BerdySensor jointAcc;
                jointAcc.type = DOF_ACCELERATION_SENSOR;
                jointAcc.id = m_model.getJointName(idx);
                jointAcc.range = sensorRange;
                sensorOrdering.push_back(jointAcc);

            }
        }

        if (m_options.includeAllJointTorquesAsSensors)
        {
            for (DOFIndex idx = 0; idx < this->m_model.getNrOfDOFs(); idx++)
            {
                IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR,idx);
                BerdySensor jointSens;
                jointSens.type = DOF_TORQUE_SENSOR;
                jointSens.id = m_model.getJointName(idx);
                jointSens.range = sensorRange;
                sensorOrdering.push_back(jointSens);
            }
        }

        if (m_options.includeAllNetExternalWrenchesAsSensors)
        {
            for (LinkIndex idx = 0; idx < this->m_model.getNrOfLinks(); idx++)
            {
                // If this link is the (fixed) base link and the
                // berdy variant is ORIGINAL_BERDY_FIXED_BASE , then
                // the net wrench applied on the base is not part of the dynamical
                // system. Anyhow, we can still write the base wrench as a function
                // of sum of the joint wrenches of all the joints attached to the base (tipically just one)
                if (m_dynamicsTraversal.getBaseLink()->getIndex() == idx &&
                    (m_options.berdyVariant != ORIGINAL_BERDY_FIXED_BASE ||
                     !m_options.includeFixedBaseExternalWrench))
                {
                    continue;
                }
                IndexRange sensorRange = this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR, idx);
                BerdySensor linkSens;
                linkSens.type = NET_EXT_WRENCH_SENSOR;
                linkSens.id = m_model.getLinkName(idx);
                linkSens.range = sensorRange;
                sensorOrdering.push_back(linkSens);

            }
        }

        for (size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++)
        {
            IndexRange sensorRange = this->getRangeJointSensorVariable(JOINT_WRENCH_SENSOR,berdySensorsInfo.wrenchSensors[i]);

            BerdySensor jointSens;
            jointSens.type = JOINT_WRENCH_SENSOR;
            jointSens.id = m_model.getJointName(i);
            jointSens.range = sensorRange;
            sensorOrdering.push_back(jointSens);
        }


        return sensorOrdering;
    }

bool BerdyHelper::serializeDynamicVariables(LinkProperAccArray& properAccs,
                                             LinkNetTotalWrenchesWithoutGravity& netTotalWrenchesWithoutGrav,
                                             LinkNetExternalWrenches& netExtWrenches,
                                             LinkInternalWrenches& linkJointWrenches,
                                             JointDOFsDoubleArray& jointTorques,
                                             JointDOFsDoubleArray& jointAccs,
                                             VectorDynSize& d)
{
    assert(d.size() == this->getNrOfDynamicVariables());
    assert(this->m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if( this->m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
    {
        // In the original formulation, also the totalNetWrenchesWithoutGravity where part of the variables
        // And the base link is not considered amoing the dynamic variables
        for(LinkIndex linkIdx = 0; linkIdx < m_model.getNrOfLinks(); linkIdx++)
        {
            // The base variables are not estimated for ORIGINAL_BERDY_FIXED_BASE
            if( linkIdx != this->m_dynamicsTraversal.getBaseLink()->getIndex() )
            {
                setSubVector(d,getRangeLinkVariable(LINK_BODY_PROPER_ACCELERATION,linkIdx),toEigen(properAccs(linkIdx)));
                setSubVector(d,getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,linkIdx),toEigen(netTotalWrenchesWithoutGrav(linkIdx)));
                if( m_options.includeAllNetExternalWrenchesAsDynamicVariables )
                {
                    setSubVector(d,getRangeLinkVariable(NET_EXT_WRENCH,linkIdx),toEigen(netExtWrenches(linkIdx)));
                }
            }
        }

        for(JointIndex jntIdx = 0; jntIdx < m_model.getNrOfJoints(); jntIdx++)
        {
            IJointConstPtr jnt = m_model.getJoint(jntIdx);

            LinkIndex childLink = m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model,jntIdx);
            setSubVector(d,getRangeJointVariable(JOINT_WRENCH,jntIdx),toEigen(linkJointWrenches(childLink)));

            // For ORIGINAL_BERDY_FIXED_BASE, we know that every joint has 1 dof
            setSubVector(d,getRangeDOFVariable(DOF_ACCELERATION,jntIdx),jointAccs(jnt->getDOFsOffset()));
            setSubVector(d,getRangeDOFVariable(DOF_TORQUE,jntIdx),jointTorques(jnt->getDOFsOffset()));
        }
    }

    return true;
}

bool BerdyHelper::serializeDynamicVariablesComputedFromFixedBaseRNEA(JointDOFsDoubleArray& jointAccs,
                                                                     LinkNetExternalWrenches& netExtWrenches,
                                                                     VectorDynSize& d)
{
    assert(jointAccs.size() == this->m_model.getNrOfDOFs());
    assert(netExtWrenches.isConsistent(this->m_model));

    LinkInternalWrenches intWrenches(this->m_model);
    FreeFloatingGeneralizedTorques genTrqs(this->m_model);

    LinkVelArray linkVels(this->m_model);
    LinkAccArray linkProperAccs(this->m_model);


    Vector3 baseProperAcc;
    toEigen(baseProperAcc) = -toEigen(m_gravity);
    Vector3 zeroVec;
    zeroVec.zero();
    dynamicsEstimationForwardVelAccKinematics(m_model,m_dynamicsTraversal,
                                                        baseProperAcc,
                                                        zeroVec,
                                                        zeroVec,
                                                        m_jointPos,
                                                        m_jointVel,
                                                        jointAccs,
                                                        linkVels,
                                                        linkProperAccs);

    RNEADynamicPhase(m_model,m_dynamicsTraversal,
                     m_jointPos,linkVels,linkProperAccs,
                     netExtWrenches,intWrenches,genTrqs);

    // Generate the d vector of dynamical variables
    assert(d.size() == this->getNrOfDynamicVariables());

    // LinkNewInternalWrenches (necessary for the old-style berdy)
    LinkNetTotalWrenchesWithoutGravity linkNetWrenchesWithoutGravity(this->model());

    for(LinkIndex visitedLinkIndex = 0; visitedLinkIndex < this->model().getNrOfLinks(); visitedLinkIndex++)
     {
         LinkConstPtr visitedLink = this->model().getLink(visitedLinkIndex);

         const iDynTree::SpatialInertia & I = visitedLink->getInertia();
         const iDynTree::SpatialAcc     & properAcc = linkProperAccs(visitedLinkIndex);
         const iDynTree::Twist          & v = linkVels(visitedLinkIndex);
         linkNetWrenchesWithoutGravity(visitedLinkIndex) = I*properAcc + v*(I*v);
     }

     return serializeDynamicVariables(linkProperAccs,linkNetWrenchesWithoutGravity,netExtWrenches,
                                      intWrenches,genTrqs.jointTorques(),jointAccs,d);
}


bool BerdyHelper::serializeSensorVariables(SensorsMeasurements& sensMeas,
                                           LinkNetExternalWrenches& netExtWrenches,
                                           JointDOFsDoubleArray& jointTorques,
                                           JointDOFsDoubleArray& jointAccs,
                                           LinkInternalWrenches& linkJointWrenches,
                                           VectorDynSize& y)
{
    bool ret=true;
    assert(y.size() == this->getNrOfSensorsMeasurements());
    assert(this->m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    bool ok = sensMeas.toVector(realSensorMeas);
    ret = ret && ok;

    // The first part of the sensor vector is exactly the serialization of the sensMeas object
    IndexRange ran;
    ran.offset = 0;
    ran.size = realSensorMeas.size();
    setSubVector(y,ran,toEigen(realSensorMeas));

    // Then we need to follow the serialization used in computeBerdySensorMatrices

     ////////////////////////////////////////////////////////////////////////
    ///// JOINT ACCELERATIONS
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllJointAccelerationsAsSensors )
    {
        for(DOFIndex idx = 0; idx< this->m_model.getNrOfDOFs(); idx++)
        {
            IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_ACCELERATION_SENSOR,idx);

            setSubVector(y,sensorRange,jointAccs(idx));
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT TORQUES
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllJointTorquesAsSensors )
    {
        for(DOFIndex idx = 0; idx< this->m_model.getNrOfDOFs(); idx++)
        {
            IndexRange sensorRange = this->getRangeDOFSensorVariable(DOF_TORQUE_SENSOR,idx);

            setSubVector(y,sensorRange,jointTorques(idx));
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// NET EXTERNAL WRENCHES ACTING ON LINKS
    ////////////////////////////////////////////////////////////////////////
    if( m_options.includeAllNetExternalWrenchesAsSensors )
    {
        for(LinkIndex idx = 0; idx < this->m_model.getNrOfLinks(); idx++)
        {
            if( !(m_options.berdyVariant == ORIGINAL_BERDY_FIXED_BASE &&
                  m_dynamicsTraversal.getBaseLink()->getIndex() == idx) ||
                 m_options.includeFixedBaseExternalWrench  )
            {
                IndexRange sensorRange = this->getRangeLinkSensorVariable(NET_EXT_WRENCH_SENSOR,idx);

                setSubVector(y,sensorRange,toEigen(netExtWrenches(idx)));
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////
    ///// JOINT WRENCHES
    ////////////////////////////////////////////////////////////////////////
    for(size_t i = 0; i < this->berdySensorsInfo.wrenchSensors.size(); i++)
    {
        IndexRange sensorRange = this->getRangeJointSensorVariable(JOINT_WRENCH_SENSOR,berdySensorsInfo.wrenchSensors[i]);

        LinkIndex childLink = m_dynamicsTraversal.getChildLinkIndexFromJointIndex(m_model,berdySensorsInfo.wrenchSensors[i]);
        setSubVector(y,sensorRange,toEigen(linkJointWrenches(childLink)));
    }


    return ret;
}

/*
std::string BerdyHelper::getDescriptionOfDynamicVariables()
{
    std::stringstream ss;
}*/





}

