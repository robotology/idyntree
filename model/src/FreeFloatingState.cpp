/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace iDynTree
{

FreeFloatingPos::FreeFloatingPos(const Model& model)
{
    resize(model);
}

void FreeFloatingPos::resize(const Model& model)
{
    this->m_jointPos.resize(model.getNrOfPosCoords());
}


Transform& FreeFloatingPos::worldBasePos()
{
    return this->m_worldBasePos;
}

IRawVector & FreeFloatingPos::jointPos()
{
    return this->m_jointPos;
}

const Transform& FreeFloatingPos::worldBasePos() const
{
    return this->m_worldBasePos;
}

const IRawVector & FreeFloatingPos::jointPos() const
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

FreeFloatingPosVelAcc::FreeFloatingPosVelAcc(const Model& model)
{
    resize(model);
}

LinkPosVelAcc& FreeFloatingPosVelAcc::basePosVelAcc()
{
    return this->m_basePosVelAcc;
}


const LinkPosVelAcc& FreeFloatingPosVelAcc::basePosVelAcc() const
{
    return this->m_basePosVelAcc;
}

IRawVector& FreeFloatingPosVelAcc::jointPos()
{
    return this->m_jointPos;
}

const IRawVector& FreeFloatingPosVelAcc::jointPos() const
{
    return this->m_jointPos;
}

IRawVector& FreeFloatingPosVelAcc::jointVel()
{
    return this->m_jointVel;
}

const IRawVector& FreeFloatingPosVelAcc::jointVel() const
{
    return this->m_jointVel;
}

IRawVector& FreeFloatingPosVelAcc::jointAcc()
{
    return this->m_jointAcc;
}

const IRawVector& FreeFloatingPosVelAcc::jointAcc() const
{
    return this->m_jointAcc;
}


void FreeFloatingPosVelAcc::resize(const Model& model)
{
    this->m_jointPos.resize(model.getNrOfPosCoords());
    this->m_jointVel.resize(model.getNrOfDOFs());
    this->m_jointAcc.resize(model.getNrOfDOFs());
}

unsigned int FreeFloatingPosVelAcc::getNrOfPosCoords() const
{
    return this->m_jointPos.size();
}

unsigned int FreeFloatingPosVelAcc::getNrOfDOFs() const
{
    return this->m_jointVel.size();
}

FreeFloatingPosVelAcc::~FreeFloatingPosVelAcc()
{

}


}
