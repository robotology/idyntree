/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{

Transform& LinkPos::pos()
{
    return this->m_pos;
}

const Transform& LinkPos::pos() const
{
    return this->m_pos;
}


LinkPos::~LinkPos()
{

}

Twist& LinkVelAcc::vel()
{
    return this->m_vel;
}

SpatialAcc& LinkVelAcc::acc()
{
    return this->m_acc;
}

const Twist& LinkVelAcc::vel() const
{
    return this->m_vel;
}

const SpatialAcc& LinkVelAcc::acc() const
{
    return this->m_acc;
}

LinkVelAcc::~LinkVelAcc()
{

}

Transform& LinkPosVelAcc::pos()
{
    return this->m_pos;
}

Twist& LinkPosVelAcc::vel()
{
    return this->m_vel;
}

SpatialAcc& LinkPosVelAcc::acc()
{
    return this->m_acc;
}

const Transform& LinkPosVelAcc::pos() const
{
    return this->m_pos;
}

const Twist& LinkPosVelAcc::vel() const
{
    return this->m_vel;
}

const SpatialAcc& LinkPosVelAcc::acc() const
{
    return this->m_acc;
}

LinkPosVelAcc::~LinkPosVelAcc()
{

}


}
