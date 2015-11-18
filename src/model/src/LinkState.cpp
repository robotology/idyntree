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

const Transform& LinkPositions::operator()(const LinkIndex link) const
{
    return this->m_linkPos[link];
}

Transform& LinkPositions::operator()(const LinkIndex link)
{
    return this->m_linkPos[link];
}

LinkPositions::~LinkPositions()
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
    return this->m_linkAcc[link];
}

SpatialAcc& LinkAccArray::operator()(const LinkIndex link)
{
    return this->m_linkAcc[link];
}

void LinkAccArray::resize(const Model& model)
{
    resize(model.getNrOfLinks());
}

void LinkAccArray::resize(unsigned int nrOfLinks)
{
    this->m_linkAcc.resize(nrOfLinks);
}

unsigned int LinkAccArray::getNrOfLinks() const
{
    return this->m_linkAcc.size();
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
