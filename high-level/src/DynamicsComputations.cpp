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


#include <kdl_codyco/KDLConversions.h>
#include <kdl_codyco/utils.hpp>

#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

#include <kdl/frames.hpp>

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
    // Spatial twist of the base link, expressed in the base link
    // frame orientation and with respect to the base link origin
    // (Warning: this members is designed to work with the legacy
    //  KDL-based algorithms, and so it is set throught
    //  setRobotState with an appropriate conversion)
    KDL::Twist    m_baseSpatialAcc;

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
    std::vector<KDL::Frame> m_fwdPosKinematicsResults;

    // storage of forward velocity kinematics results
    std::vector<KDL::Twist> m_fwdVelKinematicsResults;

    // storage of forward acceleration kinematics results
    std::vector<KDL::Twist> m_fwdAccKinematicsResults;

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
    this->pimpl->m_fwdAccKinematicsResults.resize(nrOfFrames);
    int nrOfDOFs = this->pimpl->m_robot_model.getNrOfDOFs();
    this->pimpl->m_qKDL.resize(nrOfDOFs);
    this->pimpl->m_dqKDL.resize(nrOfDOFs);
    this->pimpl->m_ddqKDL.resize(nrOfDOFs);

    // set to zero
    KDL::SetToZero(this->pimpl->m_qKDL);
    KDL::SetToZero(this->pimpl->m_dqKDL);
    KDL::SetToZero(this->pimpl->m_ddqKDL);
    KDL::SetToZero(this->pimpl->m_baseSpatialTwist);
    KDL::SetToZero(this->pimpl->m_baseSpatialAcc);
    this->pimpl->m_world2base = KDL::Frame::Identity();

}

int DynamicsComputations::getFrameIndex(const std::string& frameName) const
{
    // Currently KDL::CoDyCo::UndirectedTree mixes the concepts of frames and links
    // see https://github.com/robotology/codyco-modules/issues/39
    // Once we have a proper iDynTree::Model, we can properly implement
    // the difference between frame and link
    int index = this->pimpl->m_robot_model.getLink(frameName)->getLinkIndex();
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
                                             this->pimpl->m_baseSpatialAcc,
                                             this->pimpl->m_fwdVelKinematicsResults,
                                             this->pimpl->m_fwdAccKinematicsResults));

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
        std::cerr << "DynamicsRegressorGenerator::setRobotState failed" << std::endl;
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

    return true;
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
        return iDynTree::Transform();
    }

    if( refFrameIndex >= this->getNrOfFrames() )
    {
        reportError("DynamicsComputations","getRelativeTransform","refFrameIndex out of bound");
        return iDynTree::Transform();
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
        return iDynTree::Transform();
    }

    // compute fwd kinematics (if necessary)
    this->computeFwdKinematics();

    if( !this->pimpl->m_isFwdKinematicsUpdated )
    {
        reportError("DynamicsComputations","getWorldTransform","error in computing fwd kinematics");
        return iDynTree::Transform();
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

    return iDynTree::ToiDynTree(this->pimpl->m_fwdAccKinematicsResults[frameIndex]);
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
        return SpatialInertia();
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
        return iDynTree::SpatialInertia();
    }

    KDL::RigidBodyInertia Ikdl = this->pimpl->m_robot_model.getLink(linkIndex)->getInertia();

    // \todo TODO add semantics to SpatialInertia
    return iDynTree::ToiDynTree(Ikdl);
}




}

}

