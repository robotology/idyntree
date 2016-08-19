/**
 * Copyright (C) 2015 IIT - Istituto Italiano di Tecnologia - http://www.iit.it
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 */

#include <iDynTree/HighLevel/DynamicsComputations.h>

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


#include <kdl_codyco/KDLConversions.h>
#include <kdl_codyco/utils.hpp>

#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/jacobian_loops.hpp>
#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/com_loops.hpp>
#include <kdl_codyco/momentumjacobian.hpp>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <tinyxml.h>

#include <iostream>
#include <fstream>

namespace iDynTree
{

namespace HighLevel
{

// \todo TODO find a better way to handle the world index, and
// in general to handle the key used for semantics
const int WORLD_INDEX = -100;
unsigned int DEFAULT_DYNAMICS_COMPUTATION_FRAME_INDEX=10000;
std::string DEFAULT_DYNAMICS_COMPUTATION_FRAME_NAME="iDynTreeDynCompDefaultFrame";

struct DynamicsComputations::DynamicsComputationsPrivateAttributes
{
    bool m_isModelValid;

    // State of the model
    // Frame where the reference frame is the world one
    // and the frame is the base link one
    KDL::Frame    m_world2base;
    // Spatial twist of the base link, expressed in the base link
    // frame orientation and with respect to the base link origin
    // (Warning: this members is designed to work with the legacy
    //  KDL-based algorithms, and so it is set throught
    //  setRobotState with an appropriate conversion)
    KDL::Twist    m_baseSpatialTwist;

    // Spatial acceleration of the base link, expressed in the base link
    // frame orientation and with respect to the base link origin
    // (Warning: this members is designed to work with the legacy
    //  KDL-based algorithms, and so it is set throught
    //  setRobotState with an appropriate conversion)
    KDL::Twist    m_baseSpatialAcc;

    // Proper (actual - gravity) spatial acceleration of the base link, expressed in the base link
    // frame orientation and with respect to the base link origin
    // (Warning: this members is designed to work with the legacy
    //  KDL-based algorithms, and so it is set throught
    //  setRobotState with an appropriate conversion)
    KDL::Twist    m_baseProperSpatialAcc;

    // Spatial gravity acceleration twist, expressed in world orientation
    // (the point is not important because the angular part is always zero)
    KDL::Twist    m_gravityAcc;

    KDL::JntArray m_qKDL;
    KDL::JntArray m_dqKDL;
    KDL::JntArray m_ddqKDL;

    // Model used for dynamics computations
    KDL::CoDyCo::UndirectedTree m_robot_model;

    // Traversal (i.e. visit order of the links) used for dynamics computations
    // this defines the link that is used as a floating base
    KDL::CoDyCo::Traversal m_traversal;

    // Forward kinematics data structure
    // true whenever computePosition has been called
    // since the last call to setRobotState
    bool m_isFwdKinematicsUpdated;

    // storage of forward position kinematics results
    // element i contains the world_H_i transform
    std::vector<KDL::Frame> m_fwdPosKinematicsResults;

    // storage of forward velocity kinematics results
    std::vector<KDL::Twist> m_fwdVelKinematicsResults;

    // storage of forward acceleration kinematics results
    // this is a vector of the proper (i.e. actual - gravity) spatial acceleration
    // expressed in the link frame
    std::vector<KDL::Twist> m_fwdProperAccKinematicsResults;

    // storage for external wrenches (currently just initialied to zero)
    std::vector<KDL::Wrench> m_extWrenches;

    // storage for internal wrenches computed by inverseDynamics
    std::vector<KDL::Wrench> m_intWrenches;

    // storage for torque computed by inverseDynamics
    KDL::JntArray m_torques;

    // storage for base wrench force compute by inverseDynamics
    KDL::Wrench m_baseReactionForce;

    // storage of jacobian results
    KDL::Jacobian m_jacobianBuf;

    // storage of momentum jacobian results
    KDL::CoDyCo::MomentumJacobian m_momentum_jac_buffer;
    KDL::CoDyCo::MomentumJacobian m_momentum2_jac_buffer;


    // storage of byproducts of COM computations
    std::vector<KDL::Vector> m_subtree_COM;

    // storage of byproducts of COM computations
    std::vector<double> m_subtree_mass;

    // storage of forward position kinematics results for com computations
    // element i contains the baseLink_H_i transform
    std::vector<KDL::Frame> m_baseLinkHframe;

    //Save here the joint limits, until we have a Model object
    struct JointLimit
    {
        double min;
        double max;
    };

    typedef std::map<int, JointLimit> JointLimitMap;
    JointLimitMap jointLimits;

    DynamicsComputationsPrivateAttributes()
    {
        m_isModelValid = false;
        m_isFwdKinematicsUpdated = false;
    }
};

DynamicsComputations::DynamicsComputations():
pimpl(new DynamicsComputationsPrivateAttributes)
{
}

DynamicsComputations::DynamicsComputations(const DynamicsComputations & other):
pimpl(new DynamicsComputationsPrivateAttributes(*(other.pimpl)))
{
    // copyng the class is disabled until we get rid of the legacy implementation
    assert(false);
}

DynamicsComputations& DynamicsComputations::operator=(const DynamicsComputations& other)
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

DynamicsComputations::~DynamicsComputations()
{
    delete this->pimpl;
}

//////////////////////////////////////////////////////////////////////////////
////// Private Methods
//////////////////////////////////////////////////////////////////////////////

void DynamicsComputations::invalidateCache()
{
    this->pimpl->m_isFwdKinematicsUpdated = false;
}

void DynamicsComputations::resizeInternalDataStructures()
{
    assert(this->pimpl->m_isModelValid);

    int nrOfFrames = this->pimpl->m_robot_model.getNrOfLinks();
    this->pimpl->m_fwdPosKinematicsResults.resize(nrOfFrames);
    this->pimpl->m_fwdVelKinematicsResults.resize(nrOfFrames);
    this->pimpl->m_fwdProperAccKinematicsResults.resize(nrOfFrames);
    int nrOfDOFs = this->pimpl->m_robot_model.getNrOfDOFs();
    this->pimpl->m_qKDL.resize(nrOfDOFs);
    this->pimpl->m_dqKDL.resize(nrOfDOFs);
    this->pimpl->m_ddqKDL.resize(nrOfDOFs);
    this->pimpl->m_torques.resize(nrOfDOFs);
    this->pimpl->m_jacobianBuf.resize(nrOfDOFs+6);
    this->pimpl->m_momentum_jac_buffer.resize(nrOfDOFs+6);
    this->pimpl->m_momentum2_jac_buffer.resize(nrOfDOFs+6);
    this->pimpl->m_subtree_COM.resize(nrOfFrames);
    this->pimpl->m_subtree_mass.resize(nrOfFrames);
    this->pimpl->m_baseLinkHframe.resize(nrOfFrames);

    // set to zero
    KDL::SetToZero(this->pimpl->m_qKDL);
    KDL::SetToZero(this->pimpl->m_dqKDL);
    KDL::SetToZero(this->pimpl->m_ddqKDL);
    KDL::SetToZero(this->pimpl->m_baseSpatialTwist);
    KDL::SetToZero(this->pimpl->m_baseSpatialAcc);
    KDL::SetToZero(this->pimpl->m_baseProperSpatialAcc);
    this->pimpl->m_world2base = KDL::Frame::Identity();

    this->pimpl->m_extWrenches.resize(nrOfFrames);
    this->pimpl->m_intWrenches.resize(nrOfFrames);

    for(unsigned int frame=0; frame < nrOfFrames; frame++)
    {
        KDL::SetToZero(this->pimpl->m_extWrenches[frame]);
        KDL::SetToZero(this->pimpl->m_intWrenches[frame]);
    }

}

int DynamicsComputations::getFrameIndex(const std::string& frameName) const
{
    // Currently KDL::CoDyCo::UndirectedTree mixes the concepts of frames and links
    // see https://github.com/robotology/codyco-modules/issues/39
    // Once we have a proper iDynTree::Model, we can properly implement
    // the difference between frame and link
    KDL::CoDyCo::LinkMap::const_iterator frame = this->pimpl->m_robot_model.getLink(frameName);
    int index = -1;
    if (frame == this->pimpl->m_robot_model.getInvalidLinkIterator())
        index = -1;
    else
        index = frame->getLinkIndex();
    reportErrorIf(index < 0, "DynamicsComputations::getFrameIndex", "requested frameName not found in model");
    return index;
}

std::string DynamicsComputations::getFrameName(int frameIndex) const
{
    return this->pimpl->m_robot_model.getLink(frameIndex)->getName();
}

void DynamicsComputations::computeFwdKinematics()
{
    if( this->pimpl->m_isFwdKinematicsUpdated )
    {
        return;
    }

    // Compute position kinematics
    bool ok =
     (0 == KDL::CoDyCo::getFramesLoop(this->pimpl->m_robot_model,
                                      this->pimpl->m_qKDL,
                                      this->pimpl->m_traversal,
                                      this->pimpl->m_fwdPosKinematicsResults,
                                      this->pimpl->m_world2base));

     // Compute velocity and acceleration kinematics
     ok = ok &&
        (0 == KDL::CoDyCo::rneaKinematicLoop(this->pimpl->m_robot_model,
                                             this->pimpl->m_qKDL,
                                             this->pimpl->m_dqKDL,
                                             this->pimpl->m_ddqKDL,
                                             this->pimpl->m_traversal,
                                             this->pimpl->m_baseSpatialTwist,
                                             this->pimpl->m_baseProperSpatialAcc,
                                             this->pimpl->m_fwdVelKinematicsResults,
                                             this->pimpl->m_fwdProperAccKinematicsResults));

    this->pimpl->m_isFwdKinematicsUpdated = ok;

    return;
}

bool DynamicsComputations::loadRobotModelFromFile(const std::string& filename,
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

bool DynamicsComputations::loadRobotModelFromString(const std::string& modelString,
                                                                    const std::string& filetype)
{
    if( filetype != "urdf" )
    {
        std::cerr << "[ERROR] unknown format " << filetype <<
                     " . Currently only the urdf format is supported." << std::endl;
        return false;
    }

    bool consider_root_link_inertia = false;
    KDL::Tree local_model;
    bool ok = iDynTree::treeFromUrdfString(modelString,local_model,consider_root_link_inertia);

    this->pimpl->m_robot_model = KDL::CoDyCo::UndirectedTree(local_model);

    //TODO:
    std::vector<std::string> joint_names;
    KDL::JntArray min;
    KDL::JntArray max;
    ok = ok && iDynTree::jointPosLimitsFromUrdfString(modelString,
                                                      joint_names,
                                                      min,
                                                      max);

    if (min.rows() != max.rows() || min.rows() != joint_names.size()) {
        std::cerr << "[ERROR] error in loading joint limits" << std::endl;
        return false;
    }

    for (size_t index = 0; index < joint_names.size(); ++index) {
        int jointIndex = getJointIndex(joint_names[index]);
        if (jointIndex >= 0) {
            DynamicsComputationsPrivateAttributes::JointLimit limit;
            limit.min = min(index);
            limit.max = max(index);
            this->pimpl->jointLimits.insert(DynamicsComputationsPrivateAttributes::JointLimitMap::value_type(jointIndex, limit));
        }
    }

    if( !ok )
    {
        std::cerr << "[ERROR] error in loading robot model" << std::endl;
        return false;
    }
    else
    {
        this->pimpl->m_isModelValid = true;
        this->pimpl->m_robot_model.compute_traversal(this->pimpl->m_traversal);
        this->resizeInternalDataStructures();
        this->invalidateCache();
        return true;
    }
}



bool DynamicsComputations::isValid()
{
    return (this->pimpl->m_isModelValid);
}

std::string DynamicsComputations::getFloatingBase() const
{
    int base_link = this->pimpl->m_traversal.getBaseLink()->getLinkIndex();
    return this->pimpl->m_robot_model.getLink(base_link)->getName();
}

bool DynamicsComputations::setFloatingBase(const std::string& floatingBaseName)
{
    int retVal = this->pimpl->m_robot_model.compute_traversal(this->pimpl->m_traversal,floatingBaseName);

    if( retVal == 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////////
//// Degrees of freedom related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsComputations::getNrOfDegreesOfFreedom() const
{
    return (unsigned int)this->pimpl->m_robot_model.getNrOfDOFs();
}

std::string DynamicsComputations::getDescriptionOfDegreeOfFreedom(int dof_index)
{
    return this->pimpl->m_robot_model.getJunction(dof_index)->getName();
}

std::string DynamicsComputations::getDescriptionOfDegreesOfFreedom()
{
    std::stringstream ss;

    for(unsigned int dof = 0; dof < this->getNrOfDegreesOfFreedom(); dof++ )
    {
        ss << "DOF Index: " << dof << " Name: " <<  this->getDescriptionOfDegreeOfFreedom(dof) << std::endl;
    }

    return ss.str();
}

//////////////////////////////////////////////////////////////////////////////
//// Links related methods
//////////////////////////////////////////////////////////////////////////////

/*
unsigned int DynamicsRegressorGenerator::getNrOfLinks() const
{
    assert(false);
    return (unsigned int)this->pimpl->m_pLegacyGenerator->getNrOfDOFs();
}

std::string DynamicsRegressorGenerator::getDescriptionOfLink(int link_index)
{

}

std::string DynamicsRegressorGenerator::getDescriptionOfLinks()
{

}*/



bool DynamicsComputations::setRobotState(const VectorDynSize& q,
                                         const VectorDynSize& q_dot,
                                         const VectorDynSize& q_dotdot,
                                         const SpatialAcc& world_gravity)
{
    Transform world_T_base = Transform::Identity();
    Twist base_velocity = Twist::Zero();
    ClassicalAcc base_acceleration = ClassicalAcc::Zero();

    return setRobotState(q,q_dot,q_dotdot,
                         world_T_base,base_velocity,base_acceleration,
                         world_gravity);
}

bool DynamicsComputations::setRobotState(const VectorDynSize& q,
                                         const VectorDynSize& q_dot,
                                         const VectorDynSize& q_dotdot,
                                         const Transform& world_T_base,
                                         const Twist& base_velocity,
                                         const ClassicalAcc& base_acceleration,
                                         const SpatialAcc& world_gravity)
{
    bool ok = true;
    ok = ok && ToKDL(q,this->pimpl->m_qKDL);
    ok = ok && ToKDL(q_dot,this->pimpl->m_dqKDL);
    ok = ok && ToKDL(q_dotdot,this->pimpl->m_ddqKDL);
    this->pimpl->m_world2base = ToKDL(world_T_base);

    if( !ok )
    {
        std::cerr << "DynamicsComputations::setRobotState failed" << std::endl;
        return false;
    }

    this->invalidateCache();

    // Save gravity
    this->pimpl->m_gravityAcc = ToKDL(world_gravity);

    // Convert from the new DynamicRegressorGenerator convention to the old one
    // The base twist and acceleration are already expressed in the base origin,
    // we just need to rotate them in base orientation
    Rotation base_R_world = world_T_base.getRotation().inverse();
    Twist base_velocity_wrt_base = base_R_world*base_velocity;
    ClassicalAcc base_classical_acceleration_wrt_base = base_R_world*base_acceleration;
    SpatialAcc gravity_acceleration_wrt_base        = base_R_world*world_gravity;

    this->pimpl->m_baseSpatialTwist = ToKDL(base_velocity_wrt_base);
    KDL::Twist kdl_classical_base_acceleration = ToKDL(base_classical_acceleration_wrt_base);

    KDL::CoDyCo::conventionalToSpatialAcceleration(kdl_classical_base_acceleration,
                                                   this->pimpl->m_baseSpatialTwist,this->pimpl->m_baseSpatialAcc);

    // we save the proper acceleration of the base link
    this->pimpl->m_baseProperSpatialAcc = this->pimpl->m_baseSpatialAcc - ToKDL(gravity_acceleration_wrt_base);

    return true;
}

Transform DynamicsComputations::getWorldBaseTransform()
{
    return ToiDynTree(this->pimpl->m_world2base);
}

Twist DynamicsComputations::getBaseTwist()
{
    Rotation world_R_base = ToiDynTree(this->pimpl->m_world2base).getRotation();
    return world_R_base*ToiDynTree(this->pimpl->m_baseSpatialTwist);
}

bool DynamicsComputations::getJointPos(VectorDynSize& q)
{
    return ToiDynTree(this->pimpl->m_qKDL,q);
}

bool DynamicsComputations::getJointVel(VectorDynSize& dq)
{
    return ToiDynTree(this->pimpl->m_dqKDL,dq);
}


Transform DynamicsComputations::getRelativeTransform(const std::string& refFrameName,
                                                     const std::string& frameName)
{
    int refFrameIndex = getFrameIndex(refFrameName);
    int frameIndex = getFrameIndex(frameName);
    if( frameIndex < 0 )
    {
        reportError("DynamicsComputations","getRelativeTransform","unknown frameName");
        return Transform::Identity();
    }
    else if( refFrameIndex < 0 )
    {
        reportError("DynamicsComputations","getRelativeTransform","unknown refFrameName");
        return Transform::Identity();
    }
    else
    {
        return this->getRelativeTransform(refFrameIndex,frameIndex);
    }
}

Transform DynamicsComputations::getRelativeTransform(unsigned int refFrameIndex,
                                                     unsigned int frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getRelativeTransform","frameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    if( refFrameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getRelativeTransform","refFrameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    KDL::Frame world_H_frame = this->pimpl->m_fwdPosKinematicsResults[frameIndex];
    KDL::Frame world_H_refFrame = this->pimpl->m_fwdPosKinematicsResults[refFrameIndex];
    KDL::Frame refFrame_H_frame = world_H_refFrame.Inverse()*world_H_frame;
    iDynTree::Transform ret = iDynTree::ToiDynTree(refFrame_H_frame);

    // set semantics
    // Setting position semantics
    PositionSemantics posSem;
    posSem.setCoordinateFrame(refFrameIndex);
    posSem.setReferencePoint(refFrameIndex);
    posSem.setPoint(frameIndex);

    ret.getSemantics().setPositionSemantics(posSem);

    // Setting rotation semantics
    RotationSemantics rotSem;
    rotSem.setReferenceOrientationFrame(refFrameIndex);
    rotSem.setCoordinateFrame(refFrameIndex);
    rotSem.setOrientationFrame(frameIndex);

    ret.getSemantics().setRotationSemantics(rotSem);

    return ret;
}

Transform DynamicsComputations::getWorldTransform(std::string frameName)
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

Transform DynamicsComputations::getWorldTransform(unsigned int frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getWorldTransform","frameIndex out of bound");
        return iDynTree::Transform::Identity();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    if( !this->pimpl->m_isFwdKinematicsUpdated )
    {
        reportError("DynamicsComputations","getWorldTransform","error in computing fwd kinematics");
        return iDynTree::Transform::Identity();
    }

    iDynTree::Transform ret = iDynTree::ToiDynTree(this->pimpl->m_fwdPosKinematicsResults[frameIndex]);


    // Setting position semantics
    PositionSemantics posSem;
    posSem.setCoordinateFrame(WORLD_INDEX);
    posSem.setReferencePoint(WORLD_INDEX);
    posSem.setPoint(frameIndex);

    ret.getSemantics().setPositionSemantics(posSem);

    // Setting rotation semantics
    RotationSemantics rotSem;
    rotSem.setReferenceOrientationFrame(WORLD_INDEX);
    rotSem.setCoordinateFrame(WORLD_INDEX);
    rotSem.setOrientationFrame(frameIndex);

    ret.getSemantics().setRotationSemantics(rotSem);

    return ret;
}

unsigned int DynamicsComputations::getNrOfFrames() const
{
    // Currently KDL::CoDyCo::UndirectedTree mixes the concepts of frames and links
    // see https://github.com/robotology/codyco-modules/issues/39
    // Once we have a proper iDynTree::Model, we can properly implement
    // the difference between frame and link
    return this->pimpl->m_robot_model.getNrOfLinks();
}

//////////////////////////////////////////////////////////////////////////////
///// VELOCITY & ACCELERATION METHODS
//////////////////////////////////////////////////////////////////////////////

Twist DynamicsComputations::getFrameTwist(const std::string& frameName)
{
    int frameIndex = getFrameIndex(frameName);

    if( frameIndex < 0 )
    {
        return Twist();
    }

    return getFrameTwist(frameIndex);
}

Twist DynamicsComputations::getFrameTwist(const int frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getFrameTwist","frame index out of bounds");
        return iDynTree::Twist();
    }

    // Actually return the twist
    this->computeFwdKinematics();

    return iDynTree::ToiDynTree(this->pimpl->m_fwdVelKinematicsResults[frameIndex]);
}

Twist DynamicsComputations::getFrameTwistInWorldOrient(const std::string& frameName)
{
    return getWorldTransform(frameName).getRotation()*getFrameTwist(frameName);
}

Twist DynamicsComputations::getFrameTwistInWorldOrient(const int frameIndex)
{
    return getWorldTransform(frameIndex).getRotation()*getFrameTwist(frameIndex);
}

SpatialAcc DynamicsComputations::getFrameProperSpatialAcceleration(const std::string & frameName)
{
    int frameIndex = getFrameIndex(frameName);

    if( frameIndex < 0 )
    {
        return SpatialAcc();
    }

    return getFrameProperSpatialAcceleration(frameIndex);
}

SpatialAcc DynamicsComputations::getFrameProperSpatialAcceleration(const int frameIndex)
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getFrameProperSpatialAcceleration","frame index out of bounds");
        return iDynTree::SpatialAcc();
    }

    // Actually return the twist
    this->computeFwdKinematics();

    return iDynTree::ToiDynTree(this->pimpl->m_fwdProperAccKinematicsResults[frameIndex]);
}

bool DynamicsComputations::inverseDynamics(VectorDynSize& outTorques,
                                           Wrench& baseReactionForce)
{
    if( outTorques.size() != this->getNrOfDegreesOfFreedom() )
    {
        outTorques.resize(this->getNrOfDegreesOfFreedom());
        outTorques.zero();
    }

    this->computeFwdKinematics();

     // Compute velocity and acceleration kinematics
     bool ok =
        (0 == KDL::CoDyCo::rneaDynamicLoop(this->pimpl->m_robot_model,
                                           this->pimpl->m_qKDL,
                                           this->pimpl->m_traversal,
                                           this->pimpl->m_fwdVelKinematicsResults,
                                           this->pimpl->m_fwdProperAccKinematicsResults,
                                           this->pimpl->m_extWrenches,
                                           this->pimpl->m_intWrenches,
                                           this->pimpl->m_torques,
                                           this->pimpl->m_baseReactionForce));

    // todo \todo add semantics
    baseReactionForce = this->getWorldTransform(this->pimpl->m_traversal.getBaseLink()->getLinkIndex()).getRotation()*iDynTree::ToiDynTree(this->pimpl->m_baseReactionForce);

    ok = ok && iDynTree::ToiDynTree(this->pimpl->m_torques,outTorques);

    return ok;
}



//////////////////////////////////////////////////////////////////////////////
///// LINK METHODS
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsComputations::getNrOfLinks() const
{
    return this->pimpl->m_robot_model.getNrOfLinks();
}

int DynamicsComputations::getLinkIndex(const std::string& linkName) const
{
    int index = this->pimpl->m_robot_model.getLink(linkName)->getLinkIndex();
    reportErrorIf(index < 0, "DynamicsComputations::getLinkIndex", "requested frameName not found in model");
    return index;
}

SpatialInertia DynamicsComputations::getLinkInertia(const std::string& linkName) const
{
    int linkIndex = getLinkIndex(linkName);
    if( linkIndex < 0 )
    {
        return SpatialInertia::Zero();
    }
    else
    {
        return this->getLinkInertia(linkIndex);
    }
}

SpatialInertia DynamicsComputations::getLinkInertia(const unsigned int linkIndex) const
{
    if( linkIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getLinkInertia","linkIndex out of bound");
        return iDynTree::SpatialInertia::Zero();
    }

    KDL::RigidBodyInertia Ikdl = this->pimpl->m_robot_model.getLink(linkIndex)->getInertia();

    // \todo TODO add semantics to SpatialInertia
    return iDynTree::ToiDynTree(Ikdl);
}

bool DynamicsComputations::getFrameJacobian(const std::string& frameName,
                                            MatrixDynSize& outJacobian) const
{
    int frameIndex = getFrameIndex(frameName);

    if( frameIndex < 0 )
    {
        reportError("DynamicsComputations","getFrameJacobian","frameName unknown");
        return false;
    }

    return getFrameJacobian(frameIndex,outJacobian);
}

bool DynamicsComputations::getFrameJacobian(const unsigned int& frameIndex,
                                            MatrixDynSize& outJacobian) const
{
    if( frameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getFrameJacobian","frame index out of bounds");
        return false;
    }

    KDL::CoDyCo::getFloatingBaseJacobianLoop(this->pimpl->m_robot_model,
                                             KDL::CoDyCo::GeneralizedJntPositions(this->pimpl->m_world2base,this->pimpl->m_qKDL),
                                             this->pimpl->m_traversal,
                                             frameIndex,
                                             this->pimpl->m_jacobianBuf);

    return ToiDynTree(this->pimpl->m_jacobianBuf,outJacobian);
}

bool DynamicsComputations::getDynamicsRegressor(MatrixDynSize& outRegressor)
{
    //If the incoming matrix have the wrong number of rows/colums, resize it
    if( outRegressor.rows() != (int)(6+this->getNrOfDegreesOfFreedom()) ||
        outRegressor.cols() != (int)(10*this->getNrOfLinks()) ) {
        outRegressor.resize(6+this->getNrOfDegreesOfFreedom(),10*this->getNrOfLinks());
    }

    //Calculate the result directly in the output matrix
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > mapped_dynamics_regressor(outRegressor.data(),outRegressor.rows(),outRegressor.cols());

    Eigen::MatrixXd dynamics_regressor;
    dynamics_regressor.resize(6+this->getNrOfDegreesOfFreedom(),10*this->getNrOfLinks());

    this->computeFwdKinematics();

    KDL::CoDyCo::dynamicsRegressorLoop(this->pimpl->m_robot_model,
                                       this->pimpl->m_qKDL,
                                       this->pimpl->m_traversal,
                                       this->pimpl->m_fwdPosKinematicsResults,
                                       this->pimpl->m_fwdVelKinematicsResults,
                                       this->pimpl->m_fwdProperAccKinematicsResults,
                                       dynamics_regressor);


    mapped_dynamics_regressor = dynamics_regressor;

    return true;
}

bool DynamicsComputations::getModelDynamicsParameters(VectorDynSize& vec) const
{
    if( vec.size() != 10*this->getNrOfLinks() )
    {
        vec.resize(10*this->getNrOfLinks());
    }

    Eigen::Map< Eigen::VectorXd > mapped_vector(vec.data(),10*this->getNrOfLinks());
    Eigen::VectorXd inertial_parameters;
    inertial_parameters.resize(10*this->getNrOfLinks());

    inertialParametersVectorLoop(this->pimpl->m_robot_model,inertial_parameters);

    mapped_vector = inertial_parameters;

    return true;
}

iDynTree::Position DynamicsComputations::getCenterOfMass()
{
    KDL::Vector com_world;
    KDL::CoDyCo::GeneralizedJntPositions q_fb(this->pimpl->m_world2base,this->pimpl->m_qKDL);
    KDL::CoDyCo::getCenterOfMassLoop(this->pimpl->m_robot_model,q_fb,this->pimpl->m_traversal,this->pimpl->m_subtree_COM,this->pimpl->m_subtree_mass,com_world);

    return iDynTree::ToiDynTree(com_world);
}



bool DynamicsComputations::getCenterOfMassJacobian(iDynTree::MatrixDynSize & outJacobian)
{
    //If the incoming matrix have the wrong number of rows/colums, resize it
    if( outJacobian.rows() != (int)(3) ||
        outJacobian.cols() != (int)(6+this->getNrOfDegreesOfFreedom()) )
    {
        outJacobian.resize(3,6+this->getNrOfDegreesOfFreedom());
    }

    // Compute Jacobian for the Orin Average Velocity (the first three rows are the COM Jacobian, let's ignore for now the last three rows)
    this->pimpl->m_momentum_jac_buffer.data.setZero();
    this->pimpl->m_momentum2_jac_buffer.data.setZero();
    this->pimpl->m_jacobianBuf.data.setZero();

    // compute fwd kinematics (if necessary)
    bool ok =
     (0 == KDL::CoDyCo::getFramesLoop(this->pimpl->m_robot_model,
                                      this->pimpl->m_qKDL,
                                      this->pimpl->m_traversal,
                                      this->pimpl->m_baseLinkHframe,
                                      KDL::Frame::Identity()));

    if( !ok )
    {
        reportError("DynamicsComputations","getCenterOfMassJacobian","error in forward kinematics");
        return false;
    }

    KDL::RigidBodyInertia base_total_inertia;
    KDL::CoDyCo::getMomentumJacobianLoop(this->pimpl->m_robot_model,
                                         this->pimpl->m_qKDL,
                                         this->pimpl->m_traversal,
                                         this->pimpl->m_baseLinkHframe,
                                         this->pimpl->m_momentum_jac_buffer,
                                         this->pimpl->m_jacobianBuf,
                                         this->pimpl->m_momentum2_jac_buffer,
                                         base_total_inertia);


    this->pimpl->m_momentum_jac_buffer.changeRefFrame(KDL::Frame(this->pimpl->m_world2base.M));

    KDL::RigidBodyInertia total_inertia = KDL::Frame(this->pimpl->m_world2base.M)*base_total_inertia;

    if( total_inertia.getMass() == 0 )
    {
        std::cerr << "DynamicsComputations::getCenterOfMassJacobian error: Model has no mass " << std::endl;
        return false;
    }

    // To get the center of mass velocity, we express the momentum at the center of mass
    this->pimpl->m_momentum_jac_buffer.changeRefPoint(total_inertia.getCOG());

    this->pimpl->m_jacobianBuf.data = (this->pimpl->m_momentum_jac_buffer.data)/total_inertia.getMass();

    // Handle the incorrect computation done in getMomentumJacobianLoop
    this->pimpl->m_jacobianBuf.setColumn(0,KDL::Twist(KDL::Vector(1,0,0),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    this->pimpl->m_jacobianBuf.setColumn(1,KDL::Twist(KDL::Vector(0,1,0),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    this->pimpl->m_jacobianBuf.setColumn(2,KDL::Twist(KDL::Vector(0,0,1),KDL::Vector(0,0,0)).RefPoint(total_inertia.getCOG()));
    this->pimpl->m_jacobianBuf.setColumn(3,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(1,0,0)).RefPoint(total_inertia.getCOG()));
    this->pimpl->m_jacobianBuf.setColumn(4,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(0,1,0)).RefPoint(total_inertia.getCOG()));
    this->pimpl->m_jacobianBuf.setColumn(5,KDL::Twist(KDL::Vector(0,0,0),KDL::Vector(0,0,1)).RefPoint(total_inertia.getCOG()));

    iDynTree::toEigen(outJacobian) = this->pimpl->m_jacobianBuf.data.topRows<3>();

    return true;
}

//////////////////////////////////////////////////////////////////////////////
///// JOINT METHODS
//////////////////////////////////////////////////////////////////////////////
int DynamicsComputations::getJointIndex(const std::string &linkName)
{
    int index = -1;
    for (int i = 0; i < getNrOfDegreesOfFreedom(); i++) {
        if (this->getDescriptionOfDegreeOfFreedom(i) == linkName) {
            index = i;
            break;
        }
    }
    return index;
}

std::string DynamicsComputations::getJointName(const unsigned int jointIndex)
{
    return this->getDescriptionOfDegreeOfFreedom(jointIndex);
}


bool DynamicsComputations::getJointLimits(const std::string &jointName, double &min, double &max)
{
    assert(pimpl);

    int index = getJointIndex(jointName);
    if (index < 0) return false;

    return getJointLimits(index, min, max);
}

bool DynamicsComputations::getJointLimits(const int &jointIndex, double &min, double &max)
{
    assert(pimpl);
    if (jointIndex < 0) return false;

    DynamicsComputationsPrivateAttributes::JointLimitMap::const_iterator found;
    found = pimpl->jointLimits.find(jointIndex);
    if (found == pimpl->jointLimits.end())
        return false;
    min = found->second.min;
    max = found->second.max;
    return true;
}

}

}


