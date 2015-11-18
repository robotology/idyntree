/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/Model.h>

#include <cassert>

namespace iDynTree
{

Traversal::Traversal()
{
    reset(0,0);
}

Traversal::Traversal(const Traversal& other)
{
    assert(false);
}

Traversal& Traversal::operator=(const Traversal& other)
{
    assert(false);

    return *this;
}

Traversal::~Traversal()
{
    reset(0,0);
}

bool Traversal::reset(unsigned int nrOfVisitedLinks, Model& model)
{
    return reset(nrOfVisitedLinks,model.getNrOfLinks());
}


bool Traversal::reset(unsigned int nrOfVisitedLinks, unsigned int nrOfLinksInModel)
{
    this->links.resize(nrOfVisitedLinks);
    this->parents.resize(nrOfVisitedLinks);
    this->toParentJoints.resize(nrOfVisitedLinks);
    this->linkIndexToTraversalIndex.resize(nrOfLinksInModel);

    return true;
}

bool Traversal::setTraversalElement(unsigned int traversalIndex, const Link * link,
                                    const IJoint * jointToParent, const Link * parentLink)
{
    if( traversalIndex < 0 ||
        traversalIndex >= getNrOfVisitedLinks() )
    {
        return false;
    }

    this->links[traversalIndex] = link;
    this->toParentJoints[traversalIndex] = jointToParent;
    this->parents[traversalIndex] = parentLink;

    this->linkIndexToTraversalIndex[link->getIndex()] = traversalIndex;

    return true;
}

unsigned int Traversal::getNrOfVisitedLinks() const
{
    return this->links.size();
}

const Link* Traversal::getLink(unsigned int traversalIndex) const
{
    return this->links[traversalIndex];
}

const IJoint* Traversal::getParentJoint(unsigned int traversalIndex) const
{
    return this->toParentJoints[traversalIndex];
}

const Link* Traversal::getParentLink(unsigned int traversalIndex) const
{
    return this->parents[traversalIndex];
}

const IJoint* Traversal::getParentJointFromLinkIndex(const LinkIndex linkIndex) const
{
    return this->toParentJoints[linkIndexToTraversalIndex[linkIndex]];
}

const Link* Traversal::getParentLinkFromLinkIndex(const LinkIndex linkIndex) const
{
    return this->parents[linkIndexToTraversalIndex[linkIndex]];
}




}
