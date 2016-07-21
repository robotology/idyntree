/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Estimation/BerdyHelper.h>

#include <iDynTree/Core/EigenHelpers.h>

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



BerdyHelper::BerdyHelper(): m_areModelAndSensorsValid(false),
                              m_kinematicsUpdated(false),
                              m_nrOfDynamicalVariables(0),
                              m_nrOfDynamicEquations(0),
                              m_nrOfSensorsMeasurements(0)
{

}

bool BerdyHelper::init(const Model& model,
                        const SensorsList& sensors,
                        const BerdyVariants variant)
{
    // Reset the class
    m_kinematicsUpdated = false;
    m_areModelAndSensorsValid = false;

    m_model = model;
    m_sensors = sensors;
    m_berdyVariant = variant;

    m_model.computeFullTreeTraversal(m_dynamicsTraversal);
    m_kinematicTraversals.resize(m_model);
    m_jointPos.resize(m_model);
    m_jointVel.resize(m_model);
    m_linkVels.resize(m_model);

    bool res = false;
    switch(m_berdyVariant)
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
    m_nrOfDynamicalVariables = (6*4+2*1)*nrOfDOFs;
    // 19 Dynamics equations are considered in the original berdy :
    // 6 for the propagation of 6d acceleration
    // 6 for the computation of the net wrench from acc and vel
    // 6 for the computation of the joint wrench
    // 1 for the torque computations
    m_nrOfDynamicEquations   = (18+1)*nrOfDOFs;

    m_nrOfSensorsMeasurements = m_sensors.getSizeOfAllSensorsMeasurements();

    return true;
}

IndexRange BerdyHelper::getRangeOriginalBerdyFixedBase(BerdyDynamicVariablesTypes dynamicVariableType, TraversalIndex idx)
{
    IndexRange ret;
    size_t dynamicVariableSize, dynamicVariableOffset;
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
            dynamicVariableSize = 6;
            dynamicVariableOffset = 19;
            break;
        case DOF_ACCELERATION:
            dynamicVariableSize = 1;
            dynamicVariableOffset = 25;
            break;
    }

    // Note that the traversal index 0 is for the base link, that is not considered
    // in the fixed base Berdy, so we start from the link with traversal index 1
    ret.offset = 26*(idx-1)+dynamicVariableOffset;
    ret.size = dynamicVariableSize;

    return ret;
}

IndexRange BerdyHelper::getRangeLinkVariable(BerdyDynamicVariablesTypes dynamicVariableType, LinkIndex idx)
{
    assert(getBerdyVariant() == ORIGINAL_BERDY_FIXED_BASE);

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
    assert(getBerdyVariant() == ORIGINAL_BERDY_FIXED_BASE);

    if( !isJointBerdyDynamicVariable(dynamicVariableType) )
    {
        return IndexRange::InvalidRange();
    }

    return getRangeOriginalBerdyFixedBase(dynamicVariableType,getTraversalIndexFromJointIndex(m_model,m_dynamicsTraversal,idx));
}

IndexRange BerdyHelper::getRangeDOFVariable(BerdyDynamicVariablesTypes dynamicVariableType, DOFIndex idx)
{
    assert(getBerdyVariant() == ORIGINAL_BERDY_FIXED_BASE);

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

IndexRange BerdyHelper::getRangeLinkProperAccDynEq(const LinkIndex idx)
{
    IndexRange ret;

    if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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

    if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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

    if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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

    if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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
        if( !(m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex()) )
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

        if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE && parentLinkIdx == m_dynamicsTraversal.getBaseLink()->getIndex() )
        {
            // If the variant is ORIGINAL_BERDY_FIXED_BASE variant, we need to account explicitly for the gravity for links whose parent is the base
            biasAcc = biasAcc - visited_X_parent*m_gravity6D;
        }

        setSubVector(bD,
                     getRangeLinkProperAccDynEq(visitedLinkIdx),
                     toEigen(biasAcc));


        if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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

                         std::cerr << " m_linkVels(visitedLinkIdx) " << m_linkVels(visitedLinkIdx).toString() << std::endl;
                         std::cerr << " visitedLink->getInertia() " << visitedLink->getInertia().asMatrix().toString() << std::endl;
                         std::cerr << " visitedLink->getInertia()*m_linkVels(visitedLinkIdx) " << (visitedLink->getInertia()*m_linkVels(visitedLinkIdx)).toString() << std::endl;

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
        setSubMatrixToMinusIdentity(D,
                                    getRangeJointWrench(toParentJoint->getIndex()),
                                    getRangeLinkVariable(NET_EXT_WRENCH,visitedLinkIdx));


        if( m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
        {
            // In the ORIGINAL_BERDY_FIXED_BASE, we also add to the rows of the joint wrenches
            // the depend on the total wrenches
            setSubMatrixToIdentity(D,
                                   getRangeJointWrench(toParentJoint->getIndex()),
                                   getRangeLinkVariable(NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV,visitedLinkIdx));
        }

        if( m_berdyVariant == BERDY_FLOATING_BASE )
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
        if( m_berdyVariant == BERDY_FLOATING_BASE )
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
}

bool BerdyHelper::computeBerdySensorMatrices(MatrixDynSize& Y, VectorDynSize& bY)
{
    Y.resize(m_nrOfSensorsMeasurements,m_nrOfDynamicalVariables);
    bY.resize(m_nrOfSensorsMeasurements);
    // \todo TODO check if this is a bottleneck
    Y.zero();
    bY.zero();

    std::cerr << "computeBerdySensorsMatrix " << m_nrOfSensorsMeasurements << " " << m_nrOfDynamicalVariables << std::endl;

    // For now handle all sensor types explicitly TODO clean this up
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

        std::cerr << toEigen(sensor_X_link.asAdjointTransform()).block<3,6>(0,0) << std::endl;

        Twist vSensor = sensor_X_link*m_linkVels(parentLinkId);

        // bY for the accelerometer is the cross product of the angular part and the linear part of the body 6D velocity expressed in the sensor orientation and wrt to the
        setSubVector(bY,sensorRange, toEigen(vSensor.getAngularVec3().cross(vSensor.getLinearVec3())) );
    }

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

    return true;
}


bool BerdyHelper::initBerdyFloatingBase()
{
    bool res = false;

    //

    return res;
}



BerdyVariants BerdyHelper::getBerdyVariant() const
{
    return m_berdyVariant;
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

bool BerdyHelper::resizeBerdyMatrices(MatrixDynSize& D, VectorDynSize& bD,
                                       MatrixDynSize& Y, VectorDynSize& bY)
{
    D.resize(getNrOfDynamicEquations(),getNrOfDynamicVariables());
    bD.resize(getNrOfDynamicEquations());
    Y.resize(getNrOfSensorsMeasurements(),getNrOfDynamicVariables());
    bY.resize(getNrOfSensorsMeasurements());
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
    resizeBerdyMatrices(D,bD,Y,bY);

    bool ok;
    // Compute D matrix of dynamics equations
    ok = computeBerdyDynamicsMatrices(D,bD);

    // Compute Y matrix of sensors
    ok = computeBerdySensorMatrices(Y,bY);

    res = res && ok;

    return res;
}

bool BerdyHelper::serializeDynamicVariables(LinkProperAccArray& properAccs,
                                             LinkNetTotalWrenchesWithoutGravity& netTotalWrenchesWithoutGrav,
                                             LinkNetExternalWrenches& netExtWrenches,
                                             LinkInternalWrenches& linkJointWrenches,
                                             JointDOFsDoubleArray& jointTorques,
                                             JointDOFsDoubleArray& jointAccs,
                                             VectorDynSize& d)
{
//     d.resize(this->getNrOfDynamicVariables());

    assert(this->m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE);

    if( this->m_berdyVariant == ORIGINAL_BERDY_FIXED_BASE )
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
                setSubVector(d,getRangeLinkVariable(NET_EXT_WRENCH,linkIdx),toEigen(netExtWrenches(linkIdx)));
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



}

