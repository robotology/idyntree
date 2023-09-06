// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/Model.h>

#include <cassert>

namespace iDynTree
{
FreeFloatingPos::FreeFloatingPos()
{
    this->m_worldBasePos = Transform::Identity();
    this->m_jointPos.resize(0);
    this->m_jointPos.zero();
}

FreeFloatingPos::FreeFloatingPos(const Model& model)
{
    resize(model);
}

void FreeFloatingPos::resize(const Model& model)
{
    this->m_worldBasePos = Transform::Identity();
    this->m_jointPos.resize(model.getNrOfPosCoords());
    this->m_jointPos.zero();
}


Transform& FreeFloatingPos::worldBasePos()
{
    return this->m_worldBasePos;
}

JointPosDoubleArray & FreeFloatingPos::jointPos()
{
    return this->m_jointPos;
}

const Transform& FreeFloatingPos::worldBasePos() const
{
    return this->m_worldBasePos;
}

const JointPosDoubleArray & FreeFloatingPos::jointPos() const
{
    return this->m_jointPos;
}


unsigned int FreeFloatingPos::getNrOfPosCoords() const
{
    return this->m_jointPos.size();
}

FreeFloatingPos::~FreeFloatingPos()
{
}

FreeFloatingVel::FreeFloatingVel()
{
    this->m_baseVel.zero();
    this->m_jointVel.resize(0);
    this->m_jointVel.zero();
}


FreeFloatingVel::FreeFloatingVel(const Model& model)
{
    resize(model);
}

Twist& FreeFloatingVel::baseVel()
{
    return this->m_baseVel;
}

const Twist& FreeFloatingVel::baseVel() const
{
    return this->m_baseVel;
}

unsigned int FreeFloatingVel::getNrOfDOFs() const
{
    return this->m_jointVel.size();
}

JointDOFsDoubleArray& FreeFloatingVel::jointVel()
{
    return this->m_jointVel;
}

const JointDOFsDoubleArray& FreeFloatingVel::jointVel() const
{
    return this->m_jointVel;
}

void FreeFloatingVel::resize(const Model& model)
{
    this->m_baseVel.zero();
    this->m_jointVel.resize(model.getNrOfDOFs());
    this->m_jointVel.zero();
}

FreeFloatingVel::~FreeFloatingVel()
{

}

FreeFloatingAcc::FreeFloatingAcc()
{
    this->m_baseAcc.zero();
    this->m_jointAcc.resize(0);
    this->m_jointAcc.zero();
}

FreeFloatingAcc::FreeFloatingAcc(const Model& model)
{
    resize(model);
}

SpatialAcc& FreeFloatingAcc::baseAcc()
{
    return this->m_baseAcc;
}

const SpatialAcc& FreeFloatingAcc::baseAcc() const
{
    return this->m_baseAcc;
}

unsigned int FreeFloatingAcc::getNrOfDOFs() const
{
    return this->m_jointAcc.size();
}

JointDOFsDoubleArray& FreeFloatingAcc::jointAcc()
{
    return this->m_jointAcc;
}

const JointDOFsDoubleArray& FreeFloatingAcc::jointAcc() const
{
    return this->m_jointAcc;
}

void FreeFloatingAcc::resize(const Model& model)
{
    this->m_baseAcc.zero();
    this->m_jointAcc.resize(model.getNrOfDOFs());
    this->m_jointAcc.zero();
}

FreeFloatingAcc::~FreeFloatingAcc()
{

}

FreeFloatingGeneralizedTorques::FreeFloatingGeneralizedTorques()
{
    this->m_jointTorques.resize(0);
}


FreeFloatingGeneralizedTorques::FreeFloatingGeneralizedTorques(const Model& model)
{
    resize(model);
}

void FreeFloatingGeneralizedTorques::resize(const Model& model)
{
    this->m_jointTorques.resize(model.getNrOfDOFs());
}

Wrench& FreeFloatingGeneralizedTorques::baseWrench()
{
    return this->m_baseWrench;
}

JointDOFsDoubleArray& FreeFloatingGeneralizedTorques::jointTorques()
{
    return this->m_jointTorques;
}

const Wrench& FreeFloatingGeneralizedTorques::baseWrench() const
{
    return this->m_baseWrench;
}

const JointDOFsDoubleArray& FreeFloatingGeneralizedTorques::jointTorques() const
{
    return this->m_jointTorques;
}

unsigned int FreeFloatingGeneralizedTorques::getNrOfDOFs() const
{
    return this->m_jointTorques.size();
}

FreeFloatingGeneralizedTorques::~FreeFloatingGeneralizedTorques()
{

}


}
