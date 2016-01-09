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
    Transform identityTransform = Transform::Identity();
    this->m_linkPos.resize(nrOfLinks,identityTransform);
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
    this->resize(model.getNrOfLinks());
}

void LinkWrenches::resize(unsigned int nrOfLinks)
{
    iDynTree::Wrench zeroWrench = iDynTree::Wrench::Zero();
    this->m_linkWrenches.resize(nrOfLinks,zeroWrench);
}

bool LinkWrenches::isConsistent(const Model& model) const
{
    return (model.getNrOfLinks() == m_linkWrenches.size());
}

size_t LinkWrenches::getNrOfLinks() const
{
    return m_linkWrenches.size();
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
    /**
     * We reset the vector elements to be zero because the resize is
     * already an expensive operation that should not
     * be performed in efficient loop.
     */
    iDynTree::SpatialInertia zeroInertia;
    zeroInertia.zero();
    this->m_linkInertials.resize(nrOfLinks,zeroInertia);
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
    iDynTree::Twist zeroTwist = iDynTree::Twist::Zero();
    this->m_linkTwist.resize(nrOfLinks,zeroTwist);
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
    iDynTree::SpatialAcc zeroAcc = iDynTree::SpatialAcc::Zero();
    this->m_linkAcc.resize(nrOfLinks,zeroAcc);
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
    ArticulatedBodyInertia abi;
    abi.zero();
    this->m_linkABIs.resize(nrOfLinks,abi);
}

LinkArticulatedBodyInertias::~LinkArticulatedBodyInertias()
{

}










}
