/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Model.h>

#include <iDynTree/Core/ArticulatedBodyInertia.h>

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

const Transform& LinkPosVel::pos() const
{
    return this->m_pos;
}

Transform& LinkPosVel::pos()
{
    return this->m_pos;
}

const Twist& LinkPosVel::vel() const
{
    return this->m_vel;
}

Twist& LinkPosVel::vel()
{
    return this->m_vel;
}

LinkPosVel::~LinkPosVel()
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

LinkPositions::LinkPositions(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

LinkPositions::LinkPositions(const Model& model)
{
    resize(model);
}

void LinkPositions::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkPositions::resize(unsigned int nrOfLinks)
{
    this->m_linkPos.resize(nrOfLinks);
}

const LinkPos& LinkPositions::linkPos(const LinkIndex link) const
{
    return this->m_linkPos[link];
}

LinkPos& LinkPositions::linkPos(const LinkIndex link)
{
    return this->m_linkPos[link];
}

LinkPositions::~LinkPositions()
{
    resize(0);
}


LinkVelAccArray::LinkVelAccArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

LinkVelAccArray::LinkVelAccArray(const Model& model)
{
    resize(model);
}

void LinkVelAccArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkVelAccArray::resize(unsigned int nrOfLinks)
{
    this->m_linkState.resize(nrOfLinks);
}

const LinkVelAcc& LinkVelAccArray::linkVelAcc(const LinkIndex link) const
{
    return this->m_linkState[link];
}

LinkVelAcc& LinkVelAccArray::linkVelAcc(const LinkIndex link)
{
    return this->m_linkState[link];
}

LinkVelAccArray::~LinkVelAccArray()
{
    resize(0);
}


LinkPosVelAccArray::LinkPosVelAccArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

LinkPosVelAccArray::LinkPosVelAccArray(const Model& model)
{
    resize(model);
}

void LinkPosVelAccArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkPosVelAccArray::resize(unsigned int nrOfLinks)
{
    this->m_linkState.resize(nrOfLinks);
}

const LinkPosVelAcc& LinkPosVelAccArray::linkPosVelAcc(const LinkIndex link) const
{
    return this->m_linkState[link];
}

LinkPosVelAcc& LinkPosVelAccArray::linkPosVelAcc(const LinkIndex link)
{
    return this->m_linkState[link];
}

LinkPosVelAccArray::~LinkPosVelAccArray()
{
    resize(0);
}

LinkWrenches::LinkWrenches(const Model& model)
{
    resize(model);
}

LinkWrenches::LinkWrenches(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

void LinkWrenches::resize(const Model& model)
{
    this->m_linkWrenches.resize(model.getNrOfLinks());
}

void LinkWrenches::resize(unsigned int nrOfLinks)
{
    this->m_linkWrenches.resize(nrOfLinks);
}

Wrench& LinkWrenches::operator()(const LinkIndex link)
{
    return this->m_linkWrenches[link];
}

const Wrench& LinkWrenches::operator()(const LinkIndex link) const
{
    return this->m_linkWrenches[link];
}

LinkWrenches::~LinkWrenches()
{
    resize(0);
}

LinkInertias::LinkInertias(unsigned int nrOfLinks)
{
    this->resize(nrOfLinks);
}

LinkInertias::LinkInertias(const Model& model)
{
    this->resize(model);
}

void LinkInertias::resize(const Model& model)
{
    this->resize(model.getNrOfLinks());
}

void LinkInertias::resize(unsigned int nrOfLinks)
{
    this->m_linkInertials.resize(nrOfLinks);
}

SpatialInertia& LinkInertias::operator()(const LinkIndex link)
{
    return this->m_linkInertials[(size_t)link];
}

const SpatialInertia& LinkInertias::operator()(const LinkIndex link) const
{
    return this->m_linkInertials[(size_t)link];
}


LinkInertias::~LinkInertias()
{

}

LinkVelArray::LinkVelArray(const Model& model)
{
    resize(model);
}

LinkVelArray::LinkVelArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const Twist& LinkVelArray::operator()(const LinkIndex link) const
{
    return this->m_linkTwist[link];
}

Twist& LinkVelArray::operator()(const LinkIndex link)
{
    return this->m_linkTwist[link];
}

void LinkVelArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkVelArray::resize(unsigned int nrOfLinks)
{
    this->m_linkTwist.resize(nrOfLinks);
}

LinkVelArray::~LinkVelArray()
{

}

LinkAccArray::LinkAccArray(const Model& model)
{
    resize(model);
}

LinkAccArray::LinkAccArray(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const SpatialAcc& LinkAccArray::operator()(const LinkIndex link) const
{
    return this->m_linkTwist[link];
}

SpatialAcc& LinkAccArray::operator()(const LinkIndex link)
{
    return this->m_linkTwist[link];
}

void LinkAccArray::resize(const Model& model)
{
    resize(model.getNrOfDOFs());
}

void LinkAccArray::resize(unsigned int nrOfLinks)
{
    this->m_linkTwist.resize(nrOfLinks);
}

LinkAccArray::~LinkAccArray()
{

}

LinkArticulatedBodyInertias::LinkArticulatedBodyInertias(const Model& model)
{
    resize(model);
}

LinkArticulatedBodyInertias::LinkArticulatedBodyInertias(unsigned int nrOfLinks)
{
    resize(nrOfLinks);
}

const ArticulatedBodyInertia& LinkArticulatedBodyInertias::operator()(const LinkIndex link) const
{
    return this->m_linkABIs[link];
}

ArticulatedBodyInertia& LinkArticulatedBodyInertias::operator()(const LinkIndex link)
{
    return this->m_linkABIs[link];
}

void LinkArticulatedBodyInertias::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkArticulatedBodyInertias::resize(unsigned int nrOfLinks)
{
    this->m_linkABIs.resize(nrOfLinks);
}

LinkArticulatedBodyInertias::~LinkArticulatedBodyInertias()
{

}










}
