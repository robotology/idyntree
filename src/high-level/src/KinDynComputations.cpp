/**
 * Copyright (C) 2015 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/Wrench.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/Jacobians.h>

#include <iDynTree/ModelIO/URDFModelImport.h>

#include <cassert>
#include <iostream>
#include <fstream>

namespace iDynTree
{

// \todo TODO find a better way to handle the world index, and
// in general to handle the key used for semantics
const int WORLD_INDEX = -100;
unsigned int DEFAULT_DYNAMICS_COMPUTATION_FRAME_INDEX=10000;
std::string DEFAULT_DYNAMICS_COMPUTATION_FRAME_NAME="iDynTreeDynCompDefaultFrame";

struct KinDynComputations::KinDynComputationsPrivateAttributes
{
private:
    // Disable copy constructor and copy operator (move them to = delete when we support C++11)
    KinDynComputationsPrivateAttributes(const KinDynComputationsPrivateAttributes&other)
    {
        assert(false);
    }

    KinDynComputationsPrivateAttributes& operator=(const Traversal& other)
    {
        assert(false);

        return *this;
    }


public:
    // True if the the model is valid, false otherwise.
    bool m_isModelValid;

    // Frame  velocity representaiton used by the class
    FrameVelocityRepresentation m_frameVelRepr;

    // Model used for dynamics computations
    iDynTree::Model m_robot_model;

    // Traversal (i.e. visit order of the links) used for dynamics computations
    // this defines the link that is used as a floating base
    iDynTree::Traversal m_traversal;

    // State of the model
    // Frame where the reference frame is the world one
    // and the frame is the base link one
    iDynTree::FreeFloatingPos m_pos;

    // Velocity of the floating system
    // (Warning: this members is designed to work with the low-level
    // dynamics algorithms of iDynTree , and so it always contain
    // the base velocity expressed with the BODY_FIXED representation.
    // If a different convention is used by the class, an approprate
    // conversion is performed on set/get .
    iDynTree::FreeFloatingVel m_vel;

    // 3d gravity vector, expressed with the orientation of the inertial (world) frame
    iDynTree::Vector3 m_gravityAcc;

    // 3d gravity vector, expressed with the orientation of the base link frame
    iDynTree::Vector3 m_gravityAccInBaseLinkFrame;

    // Forward kinematics data structure
    // true whenever computePosition has been called
    // since the last call to setRobotState
    bool m_isFwdKinematicsUpdated;

    // storage of forward position kinematics results
    iDynTree::LinkPositions m_linkPos;

    // storage of forward velocity kinematics results
    iDynTree::LinkVelArray m_linkVel;

    bool m_isRawMassMatrixUpdated;

    // storage of the CRBs, used to extract
    LinkCompositeRigidBodyInertias m_linkCRBIs;

    // Helper function to get the lockedInertia of the robot from the m_linkCRBIs
    const SpatialInertia & getRobotLockedInertia();

    // Process a jacobian that expects a body fixed base velocity depending on the selected FrameVelocityRepresentation
    void processOnRightSideMatrixExpectingBodyFixedModelVelocity(MatrixDynSize &mat);
    void processOnLeftSideBodyFixedBaseMomentumJacobian(MatrixDynSize & jac);
    void processOnLeftSideBodyFixedAvgVelocityJacobian(MatrixDynSize &jac);
    void processOnLeftSideBodyFixedCentroidalAvgVelocityJacobian(MatrixDynSize &jac, const FrameVelocityRepresentation & leftSideRepresentation);

    // Transform a wrench from and to body fixed and the used representation
    Wrench fromBodyFixedToUsedRepresentation(const Wrench & wrenchInBodyFixed, const Transform & inertial_X_link);
    Wrench fromUsedRepresentationToBodyFixed(const Wrench & wrenchInBodyFixed, const Transform & inertial_X_link);

    // storage of the raw output of the CRBA, used to extract
    // the mass matrix and most the centroidal quantities
    FreeFloatingMassMatrix m_rawMassMatrix;

    // Total linear and angular momentum, expressed in the world frame
    SpatialMomentum m_totalMomentum;

    // Jacobian buffer, used for intermediate computation in COM jacobian
    MatrixDynSize m_jacBuffer;

    // Bias accelerations buffers
    bool m_areBiasAccelerationsUpdated;

    // storage of bias accelerations buffers (contains the bias acceleration for the given link in body-fixed)
    LinkAccArray m_linkBiasAcc;

    // Inverse dynamics buffers

    /** Base acceleration, in body-fixed representation */
    SpatialAcc m_invDynBaseAcc;

    /** Generalized **proper** (real-gravity) acceleration, base part in body-fixed representation */
    FreeFloatingAcc m_invDynGeneralizedProperAccs;

    /** **Proper** (real-gravity) acceleration of each link, in body-fixed representation */
    LinkProperAccArray m_invDynLinkProperAccs;

    /** External wrenches, in body-fixed representation */
    LinkNetExternalWrenches m_invDynNetExtWrenches;

    /** Internal wrenches, in body-fixed representation */
    LinkInternalWrenches m_invDynInternalWrenches;

    /** Buffer of robot velocity, always set to zero for gravity computations */
    FreeFloatingVel m_invDynZeroVel;

    /** Buffer of link velocities, always set to zero for gravity computations */
    LinkVelArray m_invDynZeroLinkVel;

    KinDynComputationsPrivateAttributes()
    {
        m_isModelValid = false;
        m_frameVelRepr = MIXED_REPRESENTATION;
        m_isFwdKinematicsUpdated = false;
        m_isRawMassMatrixUpdated = false;
        m_areBiasAccelerationsUpdated = false;
    }
};

KinDynComputations::KinDynComputations():
pimpl(new KinDynComputationsPrivateAttributes)
{
}

KinDynComputations::KinDynComputations(const KinDynComputations & other)
{
    // copyng the class is disabled until we get rid of the legacy implementation
    assert(false);
}

KinDynComputations& KinDynComputations::operator=(const KinDynComputations& other)
{
    /*
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
    */
    // copyng the class is disable until we get rid of the legacy implementation
    assert(false);

    return *this;
}

KinDynComputations::~KinDynComputations()
{
    delete this->pimpl;
}

//////////////////////////////////////////////////////////////////////////////
////// Private Methods
//////////////////////////////////////////////////////////////////////////////

void KinDynComputations::invalidateCache()
{
    this->pimpl->m_isFwdKinematicsUpdated = false;
    this->pimpl->m_isRawMassMatrixUpdated = false;
}

void KinDynComputations::resizeInternalDataStructures()
{
    assert(this->pimpl->m_isModelValid);

    this->pimpl->m_pos.resize(this->pimpl->m_robot_model);
    this->pimpl->m_vel.resize(this->pimpl->m_robot_model);
    this->pimpl->m_linkPos.resize(this->pimpl->m_robot_model);
    this->pimpl->m_linkVel.resize(this->pimpl->m_robot_model);
    this->pimpl->m_linkCRBIs.resize(this->pimpl->m_robot_model);
    this->pimpl->m_rawMassMatrix.resize(this->pimpl->m_robot_model);
    this->pimpl->m_rawMassMatrix.zero();
    this->pimpl->m_jacBuffer.resize(6,6+this->pimpl->m_robot_model.getNrOfDOFs());
    this->pimpl->m_jacBuffer.zero();
    this->pimpl->m_linkBiasAcc.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynBaseAcc.zero();
    this->pimpl->m_invDynGeneralizedProperAccs.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynNetExtWrenches.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynInternalWrenches.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynLinkProperAccs.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynZeroVel.resize(this->pimpl->m_robot_model);
    this->pimpl->m_invDynZeroVel.baseVel().zero();
    this->pimpl->m_invDynZeroVel.jointVel().zero();
    this->pimpl->m_invDynZeroLinkVel.resize(this->pimpl->m_robot_model);

    for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(pimpl->m_robot_model.getNrOfLinks()); lnkIdx++)
    {
        pimpl->m_invDynZeroLinkVel(lnkIdx).zero();
    }
}

int KinDynComputations::getFrameIndex(const std::string& frameName) const
{
    int index = this->pimpl->m_robot_model.getFrameIndex(frameName);
    reportErrorIf(index < 0, "KinDynComputations::getFrameIndex", "requested frameName not found in model");
    return index;
}

std::string KinDynComputations::getFrameName(int frameIndex) const
{
    return this->pimpl->m_robot_model.getFrameName(frameIndex);
}

void KinDynComputations::computeFwdKinematics()
{
    if( this->pimpl->m_isFwdKinematicsUpdated )
    {
        return;
    }

    // Compute position and velocity kinematics
    bool ok = ForwardPosVelKinematics(this->pimpl->m_robot_model,
                                      this->pimpl->m_traversal,
                                      this->pimpl->m_pos,
                                      this->pimpl->m_vel,
                                      this->pimpl->m_linkPos,
                                      this->pimpl->m_linkVel);

    this->pimpl->m_isFwdKinematicsUpdated = ok;

    return;
}

void KinDynComputations::computeRawMassMatrixAndTotalMomentum()
{
    if( this->pimpl->m_isRawMassMatrixUpdated )
    {
        return;
    }

    // Compute raw mass matrix
    bool ok = CompositeRigidBodyAlgorithm(pimpl->m_robot_model,
                                          pimpl->m_traversal,
                                          pimpl->m_pos.jointPos(),
                                          pimpl->m_linkCRBIs,
                                          pimpl->m_rawMassMatrix);

    reportErrorIf(!ok,"KinDynComputations::computeRawMassMatrix","Error in computing mass matrix.");


    // m_linkPos and m_linkVel are used in the computation of the total momentum
    // so we need to make sure that they are updated
    this->computeFwdKinematics();

    // Compute total momentum
    ComputeLinearAndAngularMomentum(pimpl->m_robot_model,
                                    pimpl->m_linkPos,
                                    pimpl->m_linkVel,
                                    pimpl->m_totalMomentum);

    this->pimpl->m_isRawMassMatrixUpdated = ok;
}

void KinDynComputations::computeBiasAccFwdKinematics()
{
    if( this->pimpl->m_areBiasAccelerationsUpdated )
    {
        return;
    }

    // Compute body-fixed bias accelerations
    bool ok = ForwardBiasAccKinematics(pimpl->m_robot_model,
                                       pimpl->m_traversal,
                                       pimpl->m_pos,
                                       pimpl->m_vel,
                                       pimpl->m_linkVel,
                                       pimpl->m_linkBiasAcc);

    reportErrorIf(!ok,"KinDynComputations::computeBiasAccFwdKinematics","Error in computing the bias accelerations.");

    this->pimpl->m_areBiasAccelerationsUpdated = ok;
}

bool KinDynComputations::loadRobotModelFromFile(const std::string& filename,
                                                  const std::string& filetype)
{
    if( filetype != "urdf" )
    {
        std::cerr << "[ERROR] unknown format " << filetype <<
                     " . Currently only the urdf format is supported." << std::endl;
        return false;
    }

    std::ifstream ifs(filename.c_str());

    if( !ifs )
    {
        std::cerr << "[ERROR] impossible to open file " << filename << std::endl;
        return false;
    }

    std::string model_string( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

    return this->loadRobotModelFromString(model_string);
}

bool KinDynComputations::loadRobotModelFromString(const std::string& modelString,
                                                  const std::string& filetype)
{
    if( filetype != "urdf" )
    {
        std::cerr << "[ERROR] unknown format " << filetype <<
                     " . Currently only the urdf format is supported." << std::endl;
        return false;
    }

    Model model;
    bool ok = modelFromURDFString(modelString,model);

    if( !ok )
    {
        std::cerr << "[ERROR] error in loading robot model" << std::endl;
        return false;
    }
    else
    {
        return this->loadRobotModel(model);
    }
}

bool KinDynComputations::loadRobotModel(const Model& model)
{
    this->pimpl->m_robot_model = model;
    this->pimpl->m_isModelValid = true;
    this->pimpl->m_robot_model.computeFullTreeTraversal(this->pimpl->m_traversal);
    this->resizeInternalDataStructures();
    this->invalidateCache();
    return true;
}

bool KinDynComputations::isValid() const
{
    return (this->pimpl->m_isModelValid);
}

FrameVelocityRepresentation KinDynComputations::getFrameVelocityRepresentation() const
{
    return pimpl->m_frameVelRepr;
}

bool KinDynComputations::setFrameVelocityRepresentation(const FrameVelocityRepresentation frameVelRepr) const
{
    if( frameVelRepr != INERTIAL_FIXED_REPRESENTATION &&
        frameVelRepr != BODY_FIXED_REPRESENTATION &&
        frameVelRepr != MIXED_REPRESENTATION )
    {
        reportError("KinDynComputations","setFrameVelocityRepresentation","unknown frame velocity representation");
        return false;
    }

    pimpl->m_frameVelRepr = frameVelRepr;
    return true;
}

std::string KinDynComputations::getFloatingBase() const
{
    LinkIndex base_link = this->pimpl->m_traversal.getBaseLink()->getIndex();
    return this->pimpl->m_robot_model.getLinkName(base_link);
}

bool KinDynComputations::setFloatingBase(const std::string& floatingBaseName)
{
    LinkIndex newFloatingBaseLinkIndex = this->pimpl->m_robot_model.getLinkIndex(floatingBaseName);
    return this->pimpl->m_robot_model.computeFullTreeTraversal(this->pimpl->m_traversal,newFloatingBaseLinkIndex);
}

unsigned int KinDynComputations::getNrOfLinks() const
{
    return this->pimpl->m_robot_model.getNrOfLinks();
}

const Model& KinDynComputations::getRobotModel() const
{
    return this->pimpl->m_robot_model;
}

const Model& KinDynComputations::model() const
{
    return pimpl->m_robot_model;
}

//////////////////////////////////////////////////////////////////////////////
//// Degrees of freedom related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int KinDynComputations::getNrOfDegreesOfFreedom() const
{
    return (unsigned int)this->pimpl->m_robot_model.getNrOfDOFs();
}

std::string KinDynComputations::getDescriptionOfDegreeOfFreedom(int dof_index)
{
    return this->pimpl->m_robot_model.getJointName(dof_index);
}

std::string KinDynComputations::getDescriptionOfDegreesOfFreedom()
{
    std::stringstream ss;

    for(unsigned int dof = 0; dof < this->getNrOfDegreesOfFreedom(); dof++ )
    {
        ss << "DOF Index: " << dof << " Name: " <<  this->getDescriptionOfDegreeOfFreedom(dof) << std::endl;
    }

    return ss.str();
}

bool KinDynComputations::setRobotState(const VectorDynSize& q,
                                       const VectorDynSize& q_dot,
                                       const Vector3& world_gravity)
{
    Transform world_T_base = Transform::Identity();
    Twist base_velocity = Twist::Zero();

    return setRobotState(world_T_base,q,
                         base_velocity,q_dot,
                         world_gravity);
}

bool KinDynComputations::setRobotState(const Transform& world_T_base,
                                       const VectorDynSize& qj,
                                       const Twist& base_velocity,
                                       const VectorDynSize& qj_dot,
                                       const Vector3& world_gravity)
{

    bool ok = qj.size() == pimpl->m_robot_model.getNrOfPosCoords();
    if( !ok )
    {
        reportError("KinDynComputations","setRobotState","Wrong size in input joint positions");
        return false;
    }

    ok = qj_dot.size() == pimpl->m_robot_model.getNrOfDOFs();
    if( !ok )
    {
        reportError("KinDynComputations","setRobotState","Wrong size in input joint velocities");
        return false;
    }

    this->invalidateCache();

    // Save pos
    this->pimpl->m_pos.worldBasePos() = world_T_base;
    toEigen(this->pimpl->m_pos.jointPos()) = toEigen(qj);

    // Save gravity
    this->pimpl->m_gravityAcc = world_gravity;
    Rotation base_R_inertial = this->pimpl->m_pos.worldBasePos().getRotation().inverse();
    toEigen(pimpl->m_gravityAccInBaseLinkFrame) = toEigen(base_R_inertial)*toEigen(this->pimpl->m_gravityAcc);

    // Save vel
    toEigen(pimpl->m_vel.jointVel()) = toEigen(qj_dot);

    // Account for the different possible representations
    if (pimpl->m_frameVelRepr == MIXED_REPRESENTATION)
    {
        pimpl->m_vel.baseVel() = pimpl->m_pos.worldBasePos().getRotation().inverse()*base_velocity;
    }
    else if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        // Data is stored in body fixed
        pimpl->m_vel.baseVel() = base_velocity;
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        // base_X_inertial \ls^inertial v_base
        pimpl->m_vel.baseVel() = pimpl->m_pos.worldBasePos().inverse()*base_velocity;
    }

    return true;
}

bool KinDynComputations::setJointPos(const VectorDynSize& s)
{
    bool ok = (s.size() == pimpl->m_robot_model.getNrOfPosCoords());
    if( !ok )
    {
        reportError("KinDynComputations","setJointPos","Wrong size in input joint positions");
        return false;
    }

    toEigen(this->pimpl->m_pos.jointPos()) = toEigen(s);

    // Invalidate cache
    this->invalidateCache();

    return true;
}


Transform KinDynComputations::getWorldBaseTransform()
{
    return this->pimpl->m_pos.worldBasePos();
}

Twist KinDynComputations::getBaseTwist()
{
    if (pimpl->m_frameVelRepr == MIXED_REPRESENTATION)
    {
        return pimpl->m_pos.worldBasePos().getRotation()*(pimpl->m_vel.baseVel());
    }
    else if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        // Data is stored in body fixed
        return pimpl->m_vel.baseVel();
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        // inertial_X_base \ls^base v_base
        return pimpl->m_pos.worldBasePos()*(pimpl->m_vel.baseVel());
    }

    assert(false);
    return Twist::Zero();
}

bool KinDynComputations::getJointPos(VectorDynSize& q)
{
    q.resize(this->pimpl->m_robot_model.getNrOfPosCoords());
    toEigen(q) = toEigen(this->pimpl->m_pos.jointPos());
    return true;
}

bool KinDynComputations::getJointVel(VectorDynSize& dq)
{
    dq.resize(pimpl->m_robot_model.getNrOfDOFs());
    dq = this->pimpl->m_vel.jointVel();
    return true;
}

bool KinDynComputations::getModelVel(VectorDynSize& nu)
{
    nu.resize(pimpl->m_robot_model.getNrOfDOFs()+6);
    toEigen(nu).segment<6>(0) = toEigen(getBaseTwist());
    toEigen(nu).segment(6,pimpl->m_robot_model.getNrOfDOFs()) = toEigen(this->pimpl->m_vel.jointVel());

    return true;
}



Transform KinDynComputations::getRelativeTransform(const std::string& refFrameName,
                                                   const std::string& frameName)
{
    int refFrameIndex = getFrameIndex(refFrameName);
    int frameIndex = getFrameIndex(frameName);
    if( frameIndex < 0 )
    {
        reportError("KinDynComputations","getRelativeTransform","unknown frameName");
        return Transform::Identity();
    }
    else if( refFrameIndex < 0 )
    {
        reportError("KinDynComputations","getRelativeTransform","unknown refFrameName");
        return Transform::Identity();
    }
    else
    {
        return this->getRelativeTransform(refFrameIndex,frameIndex);
    }
}

Transform KinDynComputations::getRelativeTransform(const iDynTree::FrameIndex refFrameIndex,
                                                   const iDynTree::FrameIndex frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransform","frameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    if( refFrameIndex >= this->getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransform","refFrameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    Transform world_H_frame = getWorldTransform(frameIndex);
    Transform world_H_refFrame = getWorldTransform(refFrameIndex);

    Transform refFrame_H_frame = world_H_refFrame.inverse()*world_H_frame;

    // Set semantics
    // Setting position semantics
    PositionSemantics posSem;
    posSem.setCoordinateFrame(refFrameIndex);
    posSem.setReferencePoint(refFrameIndex);
    posSem.setPoint(frameIndex);

    refFrame_H_frame.getSemantics().setPositionSemantics(posSem);

    // Setting rotation semantics
    RotationSemantics rotSem;
    rotSem.setReferenceOrientationFrame(refFrameIndex);
    rotSem.setCoordinateFrame(refFrameIndex);
    rotSem.setOrientationFrame(frameIndex);

    refFrame_H_frame.getSemantics().setRotationSemantics(rotSem);

    return refFrame_H_frame;
}

Transform KinDynComputations::getRelativeTransformExplicit(const iDynTree::FrameIndex refFrameOriginIndex,
                                                           const iDynTree::FrameIndex refFrameOrientationIndex,
                                                           const iDynTree::FrameIndex    frameOriginIndex,
                                                           const iDynTree::FrameIndex    frameOrientationIndex)
{
    if( refFrameOriginIndex >= this->pimpl->m_robot_model.getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransformExplicit","refFrameOriginIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    if( refFrameOrientationIndex >= this->pimpl->m_robot_model.getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransformExplicit","refFrameOrientationIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    if( frameOriginIndex >= this->pimpl->m_robot_model.getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransformExplicit","frameOriginIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    if( frameOrientationIndex >= this->pimpl->m_robot_model.getNrOfFrames() )
    {
        reportError("KinDynComputations","getRelativeTransformExplicit","frameOrientationIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();


    // This part can be probably made more efficient, but unless a need for performance
    // arise I prefer it to be readable for now

    Transform world_H_refFrameOrientation = getWorldTransform(refFrameOrientationIndex);
    Transform world_H_framOrientation = getWorldTransform(frameOrientationIndex);

    // Orientation part
    // refFrameOrientation_R_frameOrientation = world_R_refFrameOrientation^{-1} * world_R_frameOrientation
    Rotation refFrameOrientation_R_frameOrientation = world_H_refFrameOrientation.getRotation().inverse()*world_H_framOrientation.getRotation();

    // Position part
    // refFrameOrientation_p_refFrameOrigin_frameOrigin =
    //      refFrameOrientation_R_refFramePosition * refFramePosition_p_refFramePositon_framePosition
    Rotation refFrameOrientation_R_refFramePosition = getRelativeTransform(refFrameOrientationIndex,refFrameOriginIndex).getRotation();
    Position refFrameOrientation_p_refFrameOrigin_frameOrigin =
        refFrameOrientation_R_refFramePosition*(this->getRelativeTransform(refFrameOriginIndex,frameOriginIndex).getPosition());

    return Transform(refFrameOrientation_R_frameOrientation,refFrameOrientation_p_refFrameOrigin_frameOrigin);
}

Transform KinDynComputations::getWorldTransform(std::string frameName)
{
    int frameIndex = getFrameIndex(frameName);
    if( frameIndex < 0 )
    {
        return Transform::Identity();
    }
    else
    {
        return getWorldTransform(frameIndex);
    }
}

Transform KinDynComputations::getWorldTransform(const FrameIndex frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("KinDynComputations","getWorldTransform","frameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    if( !this->pimpl->m_isFwdKinematicsUpdated )
    {
        reportError("KinDynComputations","getWorldTransform","error in computing fwd kinematics");
        return iDynTree::Transform::Identity();
    }


    iDynTree::Transform world_H_frame;

    // If the frame is associated to a link,
    // then return directly the content in linkPos
    if( this->pimpl->m_robot_model.isValidLinkIndex(frameIndex) )
    {
        world_H_frame = this->pimpl->m_linkPos(frameIndex);
    }
    else
    {
        // otherwise we extract from the result of position kinematics
        // the transform between the world and the link at which the
        // frame is attached
        iDynTree::Transform world_H_link =
            this->pimpl->m_linkPos(this->pimpl->m_robot_model.getFrameLink(frameIndex));
        iDynTree::Transform link_H_frame =
            this->pimpl->m_robot_model.getFrameTransform(frameIndex);

        world_H_frame = world_H_link*link_H_frame;
    }

    // Setting position semantics
    PositionSemantics posSem;
    posSem.setCoordinateFrame(WORLD_INDEX);
    posSem.setReferencePoint(WORLD_INDEX);
    posSem.setPoint(frameIndex);

    world_H_frame.getSemantics().setPositionSemantics(posSem);

    // Setting rotation semantics
    RotationSemantics rotSem;
    rotSem.setReferenceOrientationFrame(WORLD_INDEX);
    rotSem.setCoordinateFrame(WORLD_INDEX);
    rotSem.setOrientationFrame(frameIndex);

    world_H_frame.getSemantics().setRotationSemantics(rotSem);

    return world_H_frame;
}

unsigned int KinDynComputations::getNrOfFrames() const
{
    return this->pimpl->m_robot_model.getNrOfFrames();
}

Twist KinDynComputations::getFrameVel(const std::string& frameName)
{
    return getFrameVel(getFrameIndex(frameName));
}

Twist KinDynComputations::getFrameVel(const FrameIndex frameIdx)
{
    if (!pimpl->m_robot_model.isValidFrameIndex(frameIdx))
    {
        reportError("KinDynComputations","getFrameVel","Frame index out of bounds");
        return Twist::Zero();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    // Compute frame body-fixed velocity
    Transform frame_X_link = pimpl->m_robot_model.getFrameTransform(frameIdx).inverse();

    Twist v_frame_body_fixed = frame_X_link*pimpl->m_linkVel(pimpl->m_robot_model.getFrameLink(frameIdx));

    if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return v_frame_body_fixed;
    }
    else
    {
        // To convert the twist to a mixed or inertial representation, we need world_H_frame
        Transform world_H_frame = getWorldTransform(frameIdx);

        if (pimpl->m_frameVelRepr == MIXED_REPRESENTATION )
        {
            return (world_H_frame.getRotation())*v_frame_body_fixed;
        }
        else
        {
            assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
            return world_H_frame*v_frame_body_fixed;
        }
    }

}


bool KinDynComputations::getFrameFreeFloatingJacobian(const std::string& frameName,
                                          MatrixDynSize& outJacobian)
{
    return getFrameFreeFloatingJacobian(getFrameIndex(frameName),outJacobian);
}

bool KinDynComputations::getFrameFreeFloatingJacobian(const FrameIndex frameIndex,
                                                      MatrixDynSize& outJacobian)
{
    if (!pimpl->m_robot_model.isValidFrameIndex(frameIndex))
    {
        reportError("KinDynComputations","getFrameJacobian","Frame index out of bounds");
        return false;
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    // Get the link to which the frame is attached
    LinkIndex jacobLink = pimpl->m_robot_model.getFrameLink(frameIndex);
    const Transform & jacobLink_H_frame = pimpl->m_robot_model.getFrameTransform(frameIndex);

    // The frame on which the jacobian is expressed is (frame,frame)
    // in the case of BODY_FIXED_REPRESENTATION, (frame,world) for MIXED_REPRESENTATION
    // and (world,world) for INERTIAL_FIXED_REPRESENTATION .
    Transform jacobFrame_X_world;

    if (pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION)
    {
        jacobFrame_X_world = Transform::Identity();
    }
    else if (pimpl->m_frameVelRepr == MIXED_REPRESENTATION)
    {
        // This is tricky.. needs to be properly documented
        Transform world_X_frame = (pimpl->m_linkPos(jacobLink)*jacobLink_H_frame);
        jacobFrame_X_world = Transform(Rotation::Identity(),-world_X_frame.getPosition());
    }
    else
    {
        assert(pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION);
        Transform world_X_frame = (pimpl->m_linkPos(jacobLink)*jacobLink_H_frame);
        jacobFrame_X_world = world_X_frame.inverse();
    }

    // To address for different representation of the base velocity, we construct the
    // baseFrame_X_jacobBaseFrame matrix
    Transform baseFrame_X_jacobBaseFrame;
    if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        baseFrame_X_jacobBaseFrame = Transform::Identity();
    }
    else if (pimpl->m_frameVelRepr == MIXED_REPRESENTATION)
    {
        Transform base_X_world = (pimpl->m_linkPos(pimpl->m_traversal.getBaseLink()->getIndex())).inverse();
        baseFrame_X_jacobBaseFrame = Transform(base_X_world.getRotation(),Position::Zero());
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        Transform world_X_base = (pimpl->m_linkPos(pimpl->m_traversal.getBaseLink()->getIndex()));
        baseFrame_X_jacobBaseFrame = world_X_base.inverse();
    }

    return FreeFloatingJacobianUsingLinkPos(pimpl->m_robot_model,pimpl->m_traversal,
                                            pimpl->m_pos.jointPos(),pimpl->m_linkPos,
                                            jacobLink,jacobFrame_X_world,baseFrame_X_jacobBaseFrame,
                                            outJacobian);
}

Vector6 KinDynComputations::getFrameBiasAcc(const std::string & frameName)
{
    return getFrameBiasAcc(getFrameIndex(frameName));
}


/**
 * Function to convert a body fixed acceleration to a mixed acceleration.
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The mixed acceleration
 */
typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

Vector6 convertBodyFixedAccelerationToMixedAcceleration(const SpatialAcc & bodyFixedAcc,
                                                        const Twist & bodyFixedVel,
                                                        const Rotation & inertial_R_body)
{
    Vector6 mixedAcceleration;

    Eigen::Map<const Eigen::Vector3d> linBodyFixedAcc(bodyFixedAcc.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedAcc(bodyFixedAcc.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linBodyFixedTwist(bodyFixedVel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedTwist(bodyFixedVel.getAngularVec3().data());

    Eigen::Map<Eigen::Vector3d> linMixedAcc(mixedAcceleration.data());
    Eigen::Map<Eigen::Vector3d> angMixedAcc(mixedAcceleration.data()+3);

    Eigen::Map<const Matrix3dRowMajor> inertial_R_body_eig(inertial_R_body.data());

    // First we account for the effect of linear/angular velocity
    linMixedAcc = inertial_R_body_eig*(linBodyFixedAcc + angBodyFixedTwist.cross(linBodyFixedTwist));

    // Angular acceleration can be copied
    angMixedAcc = inertial_R_body_eig*angMixedAcc;

    return mixedAcceleration;
}

/**
 * Function to convert mixed acceleration to body fixed acceleration
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The body fixed acceleration
 */
SpatialAcc convertMixedAccelerationToBodyFixedAcceleration(const Vector6 & mixedAcc,
                                                           const Twist & bodyFixedVel,
                                                           const Rotation & inertial_R_body)
{
    SpatialAcc bodyFixedAcc;

    Eigen::Map<const Eigen::Vector3d> linMixedAcc(mixedAcc.data());
    Eigen::Map<const Eigen::Vector3d> angMixedAcc(mixedAcc.data()+3);

    Eigen::Map<const Eigen::Vector3d> linBodyFixedTwist(bodyFixedVel.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angBodyFixedTwist(bodyFixedVel.getAngularVec3().data());

    Eigen::Map<const Matrix3dRowMajor> inertial_R_body_eig(inertial_R_body.data());

    Eigen::Map<Eigen::Vector3d> linBodyFixedAcc(bodyFixedAcc.getLinearVec3().data());
    Eigen::Map<Eigen::Vector3d> angBodyFixedAcc(bodyFixedAcc.getAngularVec3().data());

    linBodyFixedAcc = inertial_R_body_eig.transpose()*linMixedAcc - angBodyFixedTwist.cross(linBodyFixedTwist);
    angBodyFixedAcc = inertial_R_body_eig.transpose()*angMixedAcc;

    return bodyFixedAcc;
}

/**
 * Function to convert inertial acceleration to body acceleration.
 *
 * TODO refactor in a more general handling of conversion between the three
 * derivative of frame velocities (inertial, body-fixed, mixed) and the sensors
 * acceleration.
 *
 * @return The body fixed acceleration
 */
SpatialAcc convertInertialAccelerationToBodyFixedAcceleration(const Vector6 & inertialAcc,
                                                              const Transform & inertial_H_body)
{
    SpatialAcc inertialAccProperForm;
    fromEigen(inertialAccProperForm,toEigen(inertialAcc));
    return inertial_H_body.inverse()*inertialAccProperForm;
}


Vector6 KinDynComputations::getFrameBiasAcc(const FrameIndex frameIdx)
{
    if (!pimpl->m_robot_model.isValidFrameIndex(frameIdx))
    {
        reportError("KinDynComputations","getFrameBiasAcc","Frame index out of bounds");
        Vector6 zero;
        zero.zero();
        return zero;
    }

    // compute fwd kinematics and bias acceleration kinematics (if necessary)
    this->computeFwdKinematics();
    this->computeBiasAccFwdKinematics();

    // Compute frame body-fixed bias acceleration and velocity
    Transform frame_X_link = pimpl->m_robot_model.getFrameTransform(frameIdx).inverse();

    SpatialAcc bias_acc_frame_body_fixed = frame_X_link*pimpl->m_linkBiasAcc(pimpl->m_robot_model.getFrameLink(frameIdx));
    Twist      vel_frame_body_fixed      = frame_X_link*pimpl->m_linkVel(pimpl->m_robot_model.getFrameLink(frameIdx));

    // In body fixed and inertial representation, we can transform the bias acceleration with just a adjoint
    if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {

        return bias_acc_frame_body_fixed.asVector();
    }
    else
    {
        // To convert the twist to a mixed or inertial representation, we need world_H_frame
        Transform world_H_frame = getWorldTransform(frameIdx);

        if (pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION )
        {
            return (world_H_frame*bias_acc_frame_body_fixed).asVector();
        }
        else
        {
            // In the mixed case, we need to account for the non-vanishing term related to the
            // derivative of the transform between mixed and body representation
            assert(pimpl->m_frameVelRepr == MIXED_REPRESENTATION);
            return convertBodyFixedAccelerationToMixedAcceleration(bias_acc_frame_body_fixed,vel_frame_body_fixed,world_H_frame.getRotation());
        }
    }
}

Position KinDynComputations::getCenterOfMassPosition()
{
    this->computeRawMassMatrixAndTotalMomentum();

    // Extract the {}^B com from the upper left part of the inertia matrix
    iDynTree::Position base_com =  pimpl->m_linkCRBIs(pimpl->m_traversal.getBaseLink()->getIndex()).getCenterOfMass();

    // Return {}^world com = {}^world H_base \ls_base com
    return pimpl->m_pos.worldBasePos()*base_com;
}

Vector3 KinDynComputations::getCenterOfMassVelocity()
{
    this->computeRawMassMatrixAndTotalMomentum();

    // We exploit the structure of the cached total momentum to the the com velocity
    Position com_in_inertial = this->getCenterOfMassPosition();

    // Express the total momentum with the orientation of the inertial frame but in the center of mass
    SpatialMomentum m_totalMomentum_in_com_inertial = Transform(Rotation::Identity(),-com_in_inertial)*this->pimpl->m_totalMomentum;

    // The com velocity is just the linear part divided by the total mass
    double total_mass = pimpl->getRobotLockedInertia().getMass();

    Vector3 com_vel;
    toEigen(com_vel) = toEigen(m_totalMomentum_in_com_inertial.getLinearVec3())/total_mass;

    return com_vel;
}

bool KinDynComputations::getCenterOfMassJacobian(MatrixDynSize& comJacobian)
{
    this->computeRawMassMatrixAndTotalMomentum();

    comJacobian.resize(3,pimpl->m_robot_model.getNrOfDOFs()+6);

    const SpatialInertia & lockedInertia = pimpl->getRobotLockedInertia();
    Matrix6x6 invLockedInertia = lockedInertia.getInverse();

    // The first six rows of the mass matrix are the base-base average velocity jacobian
    toEigen(pimpl->m_jacBuffer) = toEigen(invLockedInertia)*toEigen(pimpl->m_rawMassMatrix).block(0,0,6,6+pimpl->m_robot_model.getNrOfDOFs());

    // Process right side of the jacobian
    pimpl->processOnRightSideMatrixExpectingBodyFixedModelVelocity(pimpl->m_jacBuffer);

    // Process the left side: we are interested in the actual "point" acceleration of the com,
    // so we get always the mixed representation
    pimpl->processOnLeftSideBodyFixedCentroidalAvgVelocityJacobian(pimpl->m_jacBuffer,MIXED_REPRESENTATION);

    // Extract the linear part, i.e. the first 3 rows
    toEigen(comJacobian) = toEigen(pimpl->m_jacBuffer).block(0,0,3,pimpl->m_robot_model.getNrOfDOFs()+6);

    return true;
}

Vector3 KinDynComputations::getCenterOfMassBiasAcc()
{
    this->computeRawMassMatrixAndTotalMomentum();
    this->computeBiasAccFwdKinematics();

    // We compute the bias of the center of mass from the bias of the total momentum derivative
    Wrench totalMomentumBiasInInertialInertial;
    ComputeLinearAndAngularMomentumDerivativeBias(pimpl->m_robot_model,
                                                  pimpl->m_linkPos,
                                                  pimpl->m_linkVel,
                                                  pimpl->m_linkBiasAcc,
                                                  totalMomentumBiasInInertialInertial);

    // The total momentum is written in the (inertial,inertial) frame
    // To get the com velocity, acceleration, we need to write it in the (com,inertial) frame
    // For the velocity we need just to multiply it for the adjoint, for the acceleration
    // we need also to account for the derivative of the adjoint term
    Position com_in_inertial = this->getCenterOfMassPosition();
    Wrench totalMomentumBiasInCOMInertial = Transform(Rotation::Identity(),-com_in_inertial)*totalMomentumBiasInInertialInertial;

    // TODO : cache com velocity
    Vector3 comVel = KinDynComputations::getCenterOfMassVelocity();

    // We account for the derivative of the transform (we can avoid this computation because we are intersted only in the linear part
    // of the momentum derivative
    // toEigen(totalMomentumBiasInCOMInertial.getAngularVec3()) =
    //    toEigen(pimpl->m_totalMomentum.getLinearVec3()).cross(toEigen(comVel));

    double total_mass = pimpl->getRobotLockedInertia().getMass();

    // Mass is constant, so we can easily divide disregarding the derivative
    Vector3 comBiasAcc;

    toEigen(comBiasAcc) = toEigen(totalMomentumBiasInCOMInertial.getLinearVec3())/total_mass;

    return comBiasAcc;
}

const SpatialInertia& KinDynComputations::KinDynComputationsPrivateAttributes::getRobotLockedInertia()
{
    return m_linkCRBIs(m_traversal.getBaseLink()->getIndex());
}

void KinDynComputations::KinDynComputationsPrivateAttributes::processOnRightSideMatrixExpectingBodyFixedModelVelocity(
        MatrixDynSize &mat)
{
    assert(mat.cols() == m_robot_model.getNrOfDOFs()+6);

    Transform baseFrame_X_newJacobBaseFrame;
    if (m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return;
    }
    else if (m_frameVelRepr == MIXED_REPRESENTATION)
    {
        Transform base_X_world = (m_linkPos(m_traversal.getBaseLink()->getIndex())).inverse();
        baseFrame_X_newJacobBaseFrame = Transform(base_X_world.getRotation(),Position::Zero());
    }
    else
    {
        assert(m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        Transform world_X_base = (m_linkPos(m_traversal.getBaseLink()->getIndex()));
        baseFrame_X_newJacobBaseFrame = world_X_base.inverse();
    }

    Matrix6x6 baseFrame_X_newJacobBaseFrame_ = baseFrame_X_newJacobBaseFrame.asAdjointTransform();

    // The first six columns of the matrix needs to be modified to account for a different representation
    // for the base velocity. This can be written as as a modification of the rows \times 6 left submatrix.
    int rows = mat.rows();
    toEigen(mat).block(0,0,rows,6) = toEigen(mat).block(0,0,rows,6)*toEigen(baseFrame_X_newJacobBaseFrame_);
}

void KinDynComputations::KinDynComputationsPrivateAttributes::processOnLeftSideBodyFixedAvgVelocityJacobian(
        MatrixDynSize &jac)
{
    assert(jac.rows() == 6);

    Transform newOutputFrame_X_oldOutputFrame;
    if (m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return;
    }
    else if (m_frameVelRepr == MIXED_REPRESENTATION)
    {
        Transform & world_X_base = m_pos.worldBasePos();
        newOutputFrame_X_oldOutputFrame = Transform(world_X_base.getRotation(),Position::Zero());
    }
    else
    {
        assert(m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        newOutputFrame_X_oldOutputFrame = m_pos.worldBasePos();
    }

    Matrix6x6 newOutputFrame_X_oldOutputFrame_ = newOutputFrame_X_oldOutputFrame.asAdjointTransform();

    toEigen(jac) = toEigen(newOutputFrame_X_oldOutputFrame_)*toEigen(jac);
}

void KinDynComputations::KinDynComputationsPrivateAttributes::processOnLeftSideBodyFixedBaseMomentumJacobian(MatrixDynSize& mat)
{
    Transform newOutputFrame_X_oldOutputFrame;
    if (m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return;
    }
    else if (m_frameVelRepr == MIXED_REPRESENTATION)
    {
        Transform & world_X_base = m_pos.worldBasePos();
        newOutputFrame_X_oldOutputFrame = Transform(world_X_base.getRotation(),Position::Zero());
    }
    else
    {
        assert(m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        newOutputFrame_X_oldOutputFrame = m_pos.worldBasePos();
    }

    Matrix6x6 newOutputFrame_X_oldOutputFrame_ = newOutputFrame_X_oldOutputFrame.asAdjointTransformWrench();

    int cols = mat.cols();
    toEigen(mat).block(0,0,6,cols) = toEigen(newOutputFrame_X_oldOutputFrame_)*toEigen(mat).block(0,0,6,cols);
}


Twist KinDynComputations::getAverageVelocity()
{
    this->computeRawMassMatrixAndTotalMomentum();

    const SpatialInertia & base_lockedInertia = pimpl->getRobotLockedInertia();
    SpatialMomentum base_momentum = pimpl->m_pos.worldBasePos().inverse()*pimpl->m_totalMomentum;
    Twist           base_averageVelocity = base_lockedInertia.applyInverse(base_momentum);

    if( pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION )
    {
        return base_averageVelocity;
    }
    else if( pimpl->m_frameVelRepr == MIXED_REPRESENTATION )
    {
        return this->pimpl->m_pos.worldBasePos().getRotation()*base_averageVelocity;
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        return this->pimpl->m_pos.worldBasePos()*base_averageVelocity;
    }

    assert(false);
}

bool KinDynComputations::getAverageVelocityJacobian(MatrixDynSize& avgVelocityJacobian)
{
    this->computeRawMassMatrixAndTotalMomentum();

    avgVelocityJacobian.resize(6,pimpl->m_robot_model.getNrOfDOFs()+6);
    const SpatialInertia & lockedInertia = pimpl->getRobotLockedInertia();
    Matrix6x6 invLockedInertia = lockedInertia.getInverse();

    // The first six rows of the mass matrix are the base-base average velocity jacobian
    toEigen(avgVelocityJacobian) = toEigen(invLockedInertia)*toEigen(pimpl->m_rawMassMatrix).block(0,0,6,6+pimpl->m_robot_model.getNrOfDOFs());

    // Handle the different representations
    pimpl->processOnRightSideMatrixExpectingBodyFixedModelVelocity(avgVelocityJacobian);
    pimpl->processOnLeftSideBodyFixedAvgVelocityJacobian(avgVelocityJacobian);

    return true;
}

void KinDynComputations::KinDynComputationsPrivateAttributes::processOnLeftSideBodyFixedCentroidalAvgVelocityJacobian(
        MatrixDynSize &jac, const FrameVelocityRepresentation & leftSideRepresentation)
{
    assert(jac.cols() == m_robot_model.getNrOfDOFs()+6);

    // Get the center of mass in the base frame
    Position vectorFromComToBaseWithRotationOfBase = PositionRaw::inverse(this->getRobotLockedInertia().getCenterOfMass());

    Transform newOutputFrame_X_oldOutputFrame;
    if (leftSideRepresentation == BODY_FIXED_REPRESENTATION)
    {
        // oldOutputFrame is B
        // newOutputFrame is G[B]

        newOutputFrame_X_oldOutputFrame =  Transform(Rotation::Identity(),vectorFromComToBaseWithRotationOfBase);
    }
    else
    {
        assert(leftSideRepresentation == INERTIAL_FIXED_REPRESENTATION ||
                       leftSideRepresentation == MIXED_REPRESENTATION);
        // oldOutputFrame is B
        // newOutputFrame is G[A]
        const Rotation & A_R_B = m_pos.worldBasePos().getRotation();
        newOutputFrame_X_oldOutputFrame = Transform(m_pos.worldBasePos().getRotation(),A_R_B*(vectorFromComToBaseWithRotationOfBase));
    }

    Matrix6x6 newOutputFrame_X_oldOutputFrame_ = newOutputFrame_X_oldOutputFrame.asAdjointTransform();

    toEigen(jac) = toEigen(newOutputFrame_X_oldOutputFrame_)*toEigen(jac);
}

Twist KinDynComputations::getCentroidalAverageVelocity()
{
    this->computeRawMassMatrixAndTotalMomentum();

    const SpatialInertia & base_lockedInertia = pimpl->getRobotLockedInertia();
    SpatialMomentum base_momentum = pimpl->m_pos.worldBasePos().inverse()*pimpl->m_totalMomentum;
    Twist           base_averageVelocity = base_lockedInertia.applyInverse(base_momentum);

    // Get the center of mass in the base frame
    Position vectorFromComToBaseWithRotationOfBase = PositionRaw::inverse(pimpl->getRobotLockedInertia().getCenterOfMass());

    Transform newOutputFrame_X_oldOutputFrame;
    if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        // oldOutputFrame is B
        // newOutputFrame is G[B]

        newOutputFrame_X_oldOutputFrame =  Transform(Rotation::Identity(),vectorFromComToBaseWithRotationOfBase);
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION ||
               pimpl->m_frameVelRepr == MIXED_REPRESENTATION);
        // oldOutputFrame is B
        // newOutputFrame is G[A]
        const Rotation & A_R_B = pimpl->m_pos.worldBasePos().getRotation();
        newOutputFrame_X_oldOutputFrame = Transform(pimpl->m_pos.worldBasePos().getRotation(),A_R_B*(vectorFromComToBaseWithRotationOfBase));
    }

    return newOutputFrame_X_oldOutputFrame*base_averageVelocity;
}


bool KinDynComputations::getCentroidalAverageVelocityJacobian(MatrixDynSize& centroidalAvgVelocityJacobian)
{
    this->computeRawMassMatrixAndTotalMomentum();

    centroidalAvgVelocityJacobian.resize(6,pimpl->m_robot_model.getNrOfDOFs()+6);
    const SpatialInertia & lockedInertia = pimpl->getRobotLockedInertia();
    Matrix6x6 invLockedInertia = lockedInertia.getInverse();
    // The first six rows of the mass matrix are the base-base average velocity jacobian
    toEigen(centroidalAvgVelocityJacobian) = toEigen(invLockedInertia)*toEigen(pimpl->m_rawMassMatrix).block(0,0,6,6+pimpl->m_robot_model.getNrOfDOFs());

    // Handle the different representations
    pimpl->processOnRightSideMatrixExpectingBodyFixedModelVelocity(centroidalAvgVelocityJacobian);
    pimpl->processOnLeftSideBodyFixedCentroidalAvgVelocityJacobian(centroidalAvgVelocityJacobian,pimpl->m_frameVelRepr);

    return true;
}

iDynTree::SpatialMomentum KinDynComputations::getLinearAngularMomentum()
{
    this->computeRawMassMatrixAndTotalMomentum();

    SpatialMomentum base_momentum = pimpl->m_pos.worldBasePos().inverse()*pimpl->m_totalMomentum;

    if( pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION )
    {
        return base_momentum;
    }
    else if( pimpl->m_frameVelRepr == MIXED_REPRESENTATION )
    {
        return this->pimpl->m_pos.worldBasePos().getRotation()*base_momentum;
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        return this->pimpl->m_totalMomentum;
    }

    assert(false);
}

bool KinDynComputations::getLinearAngularMomentumJacobian(MatrixDynSize& linAngMomentumJacobian)
{
    this->computeRawMassMatrixAndTotalMomentum();

    linAngMomentumJacobian.resize(6,pimpl->m_robot_model.getNrOfDOFs()+6);

    toEigen(linAngMomentumJacobian) = toEigen(pimpl->m_rawMassMatrix).block(0,0,6,6+pimpl->m_robot_model.getNrOfDOFs());

    // Handle the different representations
    pimpl->processOnRightSideMatrixExpectingBodyFixedModelVelocity(linAngMomentumJacobian);
    pimpl->processOnLeftSideBodyFixedBaseMomentumJacobian(linAngMomentumJacobian);

    return true;
}

SpatialMomentum KinDynComputations::getCentroidalTotalMomentum()
{
    this->computeRawMassMatrixAndTotalMomentum();

    SpatialMomentum base_momentum = pimpl->m_pos.worldBasePos().inverse()*pimpl->m_totalMomentum;

    // Get the center of mass in the base frame
    Position vectorFromComToBaseWithRotationOfBase = PositionRaw::inverse(pimpl->getRobotLockedInertia().getCenterOfMass());

    Transform newOutputFrame_X_oldOutputFrame;
    if (pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        // oldOutputFrame is B
        // newOutputFrame is G[B]
        newOutputFrame_X_oldOutputFrame =  Transform(Rotation::Identity(),vectorFromComToBaseWithRotationOfBase);
    }
    else
    {
        assert(pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION ||
               pimpl->m_frameVelRepr == MIXED_REPRESENTATION);
        // oldOutputFrame is B
        // newOutputFrame is G[A]
        const Rotation & A_R_B = pimpl->m_pos.worldBasePos().getRotation();
        newOutputFrame_X_oldOutputFrame = Transform(pimpl->m_pos.worldBasePos().getRotation(),A_R_B*(vectorFromComToBaseWithRotationOfBase));
    }

    return newOutputFrame_X_oldOutputFrame*base_momentum;
}

bool KinDynComputations::getFreeFloatingMassMatrix(MatrixDynSize& freeFloatingMassMatrix)
{
    // Compute the body-fixed-body-fixed mass matrix, if necessary 
    this->computeRawMassMatrixAndTotalMomentum();
    
    // If the matrix has the right size, this should be inexpensive 
    freeFloatingMassMatrix.resize(pimpl->m_robot_model.getNrOfDOFs()+6,pimpl->m_robot_model.getNrOfDOFs()+6);

    toEigen(freeFloatingMassMatrix) = toEigen(pimpl->m_rawMassMatrix);
    
    // Handle the different representations
    pimpl->processOnRightSideMatrixExpectingBodyFixedModelVelocity(freeFloatingMassMatrix);
    pimpl->processOnLeftSideBodyFixedBaseMomentumJacobian(freeFloatingMassMatrix);

    // Return
    return true;
}

Wrench KinDynComputations::KinDynComputationsPrivateAttributes::fromUsedRepresentationToBodyFixed(const Wrench & wrenchInUsedRepresentation,
                                                                                                  const Transform & inertial_X_bodyFixed)
{
    if (m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return Wrench(wrenchInUsedRepresentation);
    }
    else if (m_frameVelRepr == MIXED_REPRESENTATION)
    {
        return (inertial_X_bodyFixed.getRotation().inverse())*wrenchInUsedRepresentation;
    }
    else
    {
        assert(m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        return inertial_X_bodyFixed.inverse()*wrenchInUsedRepresentation;
    }

    assert(false);
    return Wrench::Zero();
}

Wrench KinDynComputations::KinDynComputationsPrivateAttributes::fromBodyFixedToUsedRepresentation(const Wrench & wrenchInBodyFixed,
                                                                                                  const Transform & inertial_X_bodyFixed)
{
    if (m_frameVelRepr == BODY_FIXED_REPRESENTATION)
    {
        return Wrench(wrenchInBodyFixed);
    }
    else if (m_frameVelRepr == MIXED_REPRESENTATION)
    {
        return inertial_X_bodyFixed.getRotation()*wrenchInBodyFixed;
    }
    else
    {
        assert(m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION);
        return inertial_X_bodyFixed*wrenchInBodyFixed;
    }

    assert(false);
    return Wrench::Zero();
}

bool KinDynComputations::inverseDynamics(const Vector6& baseAcc,
                                         const VectorDynSize& s_ddot,
                                         const LinkNetExternalWrenches & linkExtForces,
                                               FreeFloatingGeneralizedTorques & baseForceAndJointTorques)
{
    // Needed for using pimpl->m_linkVel
    this->computeFwdKinematics();

    // Convert input base acceleration
    if( pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION )
    {
        fromEigen(pimpl->m_invDynBaseAcc,toEigen(baseAcc));
    }
    else if( pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION )
    {
        pimpl->m_invDynBaseAcc = convertInertialAccelerationToBodyFixedAcceleration(baseAcc,pimpl->m_pos.worldBasePos());
    }
    else
    {
        assert(pimpl->m_frameVelRepr == MIXED_REPRESENTATION);
        pimpl->m_invDynBaseAcc = convertMixedAccelerationToBodyFixedAcceleration(baseAcc,
                                                                                 pimpl->m_vel.baseVel(),
                                                                                 pimpl->m_pos.worldBasePos().getRotation());
    }

    // Convert input external forces
    if( pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION ||
        pimpl->m_frameVelRepr == MIXED_REPRESENTATION )
    {
        this->computeFwdKinematics();

        for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(pimpl->m_robot_model.getNrOfLinks()); lnkIdx++)
        {
            const Transform & inertialFrame_X_link = pimpl->m_linkPos(lnkIdx);
            pimpl->m_invDynNetExtWrenches(lnkIdx) = pimpl->fromUsedRepresentationToBodyFixed(linkExtForces(lnkIdx),inertialFrame_X_link);
        }
    }
    else
    {
        for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(pimpl->m_robot_model.getNrOfLinks()); lnkIdx++)
        {
            pimpl->m_invDynNetExtWrenches(lnkIdx) = linkExtForces(lnkIdx);
        }
    }

    // Prepare the vector of generalized proper accs
    pimpl->m_invDynGeneralizedProperAccs.baseAcc() = pimpl->m_invDynBaseAcc;
    toEigen(pimpl->m_invDynGeneralizedProperAccs.baseAcc().getLinearVec3()) =
        toEigen(pimpl->m_invDynBaseAcc.getLinearVec3()) - toEigen(pimpl->m_gravityAccInBaseLinkFrame);
    toEigen(pimpl->m_invDynGeneralizedProperAccs.jointAcc()) = toEigen(s_ddot);

    // Run inverse dynamics
    ForwardAccKinematics(pimpl->m_robot_model,
                         pimpl->m_traversal,
                         pimpl->m_pos,
                         pimpl->m_vel,
                         pimpl->m_invDynGeneralizedProperAccs,
                         pimpl->m_linkVel,
                         pimpl->m_invDynLinkProperAccs);

    RNEADynamicPhase(pimpl->m_robot_model,
                     pimpl->m_traversal,
                     pimpl->m_pos.jointPos(),
                     pimpl->m_linkVel,
                     pimpl->m_invDynLinkProperAccs,
                     pimpl->m_invDynNetExtWrenches,
                     pimpl->m_invDynInternalWrenches,
                     baseForceAndJointTorques);

    // Convert output base force
    baseForceAndJointTorques.baseWrench() = pimpl->fromBodyFixedToUsedRepresentation(baseForceAndJointTorques.baseWrench(),
                                                                              pimpl->m_linkPos(pimpl->m_traversal.getBaseLink()->getIndex()));

    return true;
}

bool KinDynComputations::generalizedBiasForces(FreeFloatingGeneralizedTorques & generalizedBiasForces)
{
    // Needed for using pimpl->m_linkVel
    this->computeFwdKinematics();

    // The external wrenches need to be set to zero
    pimpl->m_invDynNetExtWrenches.zero();

    // The base acceleration is "zero" in the chosen representation,
    // but the "zero" in mixed means non-zero in body-fixed, and we should account for that
    Vector6 zeroBaseAcc;
    zeroBaseAcc.zero();
    if( pimpl->m_frameVelRepr == BODY_FIXED_REPRESENTATION )
    {
        fromEigen(pimpl->m_invDynBaseAcc,toEigen(zeroBaseAcc));
    }
    else if( pimpl->m_frameVelRepr == INERTIAL_FIXED_REPRESENTATION )
    {
        pimpl->m_invDynBaseAcc = convertInertialAccelerationToBodyFixedAcceleration(zeroBaseAcc,pimpl->m_pos.worldBasePos());
    }
    else
    {
        assert(pimpl->m_frameVelRepr == MIXED_REPRESENTATION);
        pimpl->m_invDynBaseAcc = convertMixedAccelerationToBodyFixedAcceleration(zeroBaseAcc,
                                                                                 pimpl->m_vel.baseVel(),
                                                                                 pimpl->m_pos.worldBasePos().getRotation());
    }

    pimpl->m_invDynGeneralizedProperAccs.baseAcc() = pimpl->m_invDynBaseAcc;
    toEigen(pimpl->m_invDynGeneralizedProperAccs.baseAcc().getLinearVec3()) =
            toEigen(pimpl->m_invDynBaseAcc.getLinearVec3()) - toEigen(pimpl->m_gravityAccInBaseLinkFrame);
    pimpl->m_invDynGeneralizedProperAccs.jointAcc().zero();

    // Run inverse dynamics
    ForwardAccKinematics(pimpl->m_robot_model,
                         pimpl->m_traversal,
                         pimpl->m_pos,
                         pimpl->m_vel,
                         pimpl->m_invDynGeneralizedProperAccs,
                         pimpl->m_linkVel,
                         pimpl->m_invDynLinkProperAccs);

    RNEADynamicPhase(pimpl->m_robot_model,
                     pimpl->m_traversal,
                     pimpl->m_pos.jointPos(),
                     pimpl->m_linkVel,
                     pimpl->m_invDynLinkProperAccs,
                     pimpl->m_invDynNetExtWrenches,
                     pimpl->m_invDynInternalWrenches,
                     generalizedBiasForces);

    // Convert output base force
    generalizedBiasForces.baseWrench() = pimpl->fromBodyFixedToUsedRepresentation(generalizedBiasForces.baseWrench(),
                                                                           pimpl->m_linkPos(pimpl->m_traversal.getBaseLink()->getIndex()));

    return true;
}

bool KinDynComputations::generalizedGravityForces(FreeFloatingGeneralizedTorques & generalizedGravityForces)
{
    // Clear input buffers that need to be cleared
    for(LinkIndex lnkIdx = 0; lnkIdx < static_cast<LinkIndex>(pimpl->m_robot_model.getNrOfLinks()); lnkIdx++)
    {
        pimpl->m_invDynNetExtWrenches(lnkIdx).zero();
    }
    pimpl->m_invDynGeneralizedProperAccs.baseAcc().zero();
    toEigen(pimpl->m_invDynGeneralizedProperAccs.baseAcc().getLinearVec3()) =
            - toEigen(pimpl->m_gravityAccInBaseLinkFrame);
    pimpl->m_invDynGeneralizedProperAccs.jointAcc().zero();

    // Run inverse dynamics
    ForwardAccKinematics(pimpl->m_robot_model,
                         pimpl->m_traversal,
                         pimpl->m_pos,
                         pimpl->m_invDynZeroVel,
                         pimpl->m_invDynGeneralizedProperAccs,
                         pimpl->m_invDynZeroLinkVel,
                         pimpl->m_invDynLinkProperAccs);

    RNEADynamicPhase(pimpl->m_robot_model,
                     pimpl->m_traversal,
                     pimpl->m_pos.jointPos(),
                     pimpl->m_invDynZeroLinkVel,
                     pimpl->m_invDynLinkProperAccs,
                     pimpl->m_invDynNetExtWrenches,
                     pimpl->m_invDynInternalWrenches,
                     generalizedGravityForces);


    // Convert output base force
    generalizedGravityForces.baseWrench() = pimpl->fromBodyFixedToUsedRepresentation(generalizedGravityForces.baseWrench(),
                                                                              pimpl->m_linkPos(pimpl->m_traversal.getBaseLink()->getIndex()));

    return true;
}

}

