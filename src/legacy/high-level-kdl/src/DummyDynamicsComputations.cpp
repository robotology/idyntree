/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/HighLevel/DynamicsComputations.h>

void printDeprecationMessage()
{
    std::cerr << "[WARNING] The iDynTree::HighLevel::DynamicsComputations class is deprecated," << std::endl <<
              "[WARNING] and it has been superseeded by the the iDynTree::KinDynComputations class." << std::endl <<
              "[WARNING] To continue to use the iDynTree::HighLevel::DynamicsComputations class in the " << std::endl <<
              "[WARNING] bindings, please compile iDynTree with the IDYNTREE_USES_KDL CMake option."  << std::endl;

}


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
};

DynamicsComputations::DynamicsComputations():
pimpl(new DynamicsComputationsPrivateAttributes)
{
    printDeprecationMessage();
}

DynamicsComputations::DynamicsComputations(const DynamicsComputations & other):
pimpl(new DynamicsComputationsPrivateAttributes(*(other.pimpl)))
{
    // copyng the class is disabled until we get rid of the legacy implementation
    assert(false);
}

DynamicsComputations& DynamicsComputations::operator=(const DynamicsComputations& other)
{
    assert(false);

    return *this;
}

DynamicsComputations::~DynamicsComputations()
{
    delete this->pimpl;
    this->pimpl = 0;
}

//////////////////////////////////////////////////////////////////////////////
////// Private Methods
//////////////////////////////////////////////////////////////////////////////

void DynamicsComputations::invalidateCache()
{
}

void DynamicsComputations::resizeInternalDataStructures()
{
}

int DynamicsComputations::getFrameIndex(const std::string&) const
{
    printDeprecationMessage();
    return -1;
}

std::string DynamicsComputations::getFrameName(int) const
{
    printDeprecationMessage();
    return "";
}

void DynamicsComputations::computeFwdKinematics()
{
    return;
}

bool DynamicsComputations::loadRobotModelFromFile(const std::string&,
                                                  const std::string&)
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::loadRobotModelFromString(const std::string&,
                                                    const std::string&)
{
    printDeprecationMessage();
    return false;
}



bool DynamicsComputations::isValid()
{
    printDeprecationMessage();
    return false;
}

std::string DynamicsComputations::getFloatingBase() const
{
    printDeprecationMessage();
    return "";
}

bool DynamicsComputations::setFloatingBase(const std::string&)
{
    printDeprecationMessage();
    return false;
}

//////////////////////////////////////////////////////////////////////////////
//// Degrees of freedom related methods
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsComputations::getNrOfDegreesOfFreedom() const
{
    printDeprecationMessage();
    return 0;
}

std::string DynamicsComputations::getDescriptionOfDegreeOfFreedom(int dof_index)
{
    printDeprecationMessage();
    return "";
}

std::string DynamicsComputations::getDescriptionOfDegreesOfFreedom()
{
    printDeprecationMessage();
    return "";
}

bool DynamicsComputations::setRobotState(const VectorDynSize&,
                                         const VectorDynSize&,
                                         const VectorDynSize&,
                                         const SpatialAcc&)
{
    printDeprecationMessage();
    return true;
}

bool DynamicsComputations::setRobotState(const VectorDynSize& ,
                                         const VectorDynSize& ,
                                         const VectorDynSize& ,
                                         const Transform& ,
                                         const Twist& ,
                                         const ClassicalAcc& ,
                                         const SpatialAcc& )
{
    printDeprecationMessage();
    return false;
}

Transform DynamicsComputations::getWorldBaseTransform()
{
    printDeprecationMessage();
    return iDynTree::Transform::Identity();
}

Twist DynamicsComputations::getBaseTwist()
{
    printDeprecationMessage();
    return Twist::Zero();
}

bool DynamicsComputations::getJointPos(VectorDynSize&)
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getJointVel(VectorDynSize&)
{
    printDeprecationMessage();
    return false;
}


Transform DynamicsComputations::getRelativeTransform(const std::string&,
                                                     const std::string&)
{
    printDeprecationMessage();
    return Transform();
}

Transform DynamicsComputations::getRelativeTransform(unsigned int,
                                                     unsigned int)
{
    printDeprecationMessage();
    return Transform();
}

Transform DynamicsComputations::getWorldTransform(std::string )
{
    printDeprecationMessage();
    return Transform();
}

Transform DynamicsComputations::getWorldTransform(unsigned int)
{
    printDeprecationMessage();
    return Transform();
}

unsigned int DynamicsComputations::getNrOfFrames() const
{
    printDeprecationMessage();
    return 0;
}

//////////////////////////////////////////////////////////////////////////////
///// VELOCITY & ACCELERATION METHODS
//////////////////////////////////////////////////////////////////////////////

Twist DynamicsComputations::getFrameTwist(const std::string&)
{
    printDeprecationMessage();
    return Twist();
}

Twist DynamicsComputations::getFrameTwist(const int)
{
    printDeprecationMessage();
    return Twist();
}

Twist DynamicsComputations::getFrameTwistInWorldOrient(const std::string&)
{
    printDeprecationMessage();
    return Twist();
}

Twist DynamicsComputations::getFrameTwistInWorldOrient(const int)
{
    printDeprecationMessage();
    return Twist();
}

SpatialAcc DynamicsComputations::getFrameProperSpatialAcceleration(const std::string &)
{
    printDeprecationMessage();
    return SpatialAcc();
}

SpatialAcc DynamicsComputations::getFrameProperSpatialAcceleration(const int)
{
    printDeprecationMessage();
    return SpatialAcc();
}

bool DynamicsComputations::inverseDynamics(VectorDynSize&,
                                           Wrench&)
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getFreeFloatingMassMatrix(iDynTree::MatrixDynSize &)
{
    printDeprecationMessage();
    return false;
}


//////////////////////////////////////////////////////////////////////////////
///// LINK METHODS
//////////////////////////////////////////////////////////////////////////////

unsigned int DynamicsComputations::getNrOfLinks() const
{
    printDeprecationMessage();
    return 0;
}

int DynamicsComputations::getLinkIndex(const std::string&) const
{
    printDeprecationMessage();
    return -1;
}

SpatialInertia DynamicsComputations::getLinkInertia(const std::string&) const
{
    printDeprecationMessage();
    return SpatialInertia();
}

SpatialInertia DynamicsComputations::getLinkInertia(const unsigned int) const
{
    printDeprecationMessage();
    return SpatialInertia();
}

bool DynamicsComputations::getFrameJacobian(const std::string&,
                                            MatrixDynSize&) const
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getFrameJacobian(const unsigned int&,
                                            MatrixDynSize&) const
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getDynamicsRegressor(MatrixDynSize&)
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getModelDynamicsParameters(VectorDynSize&) const
{
    printDeprecationMessage();
    return false;
}

iDynTree::Position DynamicsComputations::getCenterOfMass()
{
    printDeprecationMessage();
    return iDynTree::Position::Zero();
}



bool DynamicsComputations::getCenterOfMassJacobian(iDynTree::MatrixDynSize &)
{
    printDeprecationMessage();
    return false;
}

//////////////////////////////////////////////////////////////////////////////
///// JOINT METHODS
//////////////////////////////////////////////////////////////////////////////
int DynamicsComputations::getJointIndex(const std::string &linkName)
{
    printDeprecationMessage();
    return -1;
}

std::string DynamicsComputations::getJointName(const unsigned int jointIndex)
{
    printDeprecationMessage();
    return "";
}


bool DynamicsComputations::getJointLimits(const std::string &jointName, double &min, double &max)
{
    printDeprecationMessage();
    return false;
}

bool DynamicsComputations::getJointLimits(const int &, double &, double &)
{
    printDeprecationMessage();
    return false;
}

}

}


