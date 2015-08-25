/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Traversal.h>

#include <cassert>

namespace iDynTree
{

Traversal::Traversal()
{
    reset(0);
}

Traversal::Traversal(const Traversal& other)
{
    assert(false);
}

Traversal& Traversal::operator=(const Traversal& other)
{
    assert(false);
}

Traversal::~Traversal()
{
    reset(0);
}

bool Traversal::reset(unsigned int nrOfVisitedLinks)
{
    this->links.resize(nrOfVisitedLinks);
    this->parents.resize(nrOfVisitedLinks);
    this->toParentJoints.resize(nrOfVisitedLinks);
}


bool Traversal::setTraversalElement(unsigned int traversalIndex, Link * link,
                                    IJoint * jointToParent, Link * parentLink)
{
    if( traversalIndex < 0 ||
        traversalIndex >= getNrOfVisitedLinks() )
    {
        return false;
    }

    this->links[traversalIndex] = link;
    this->toParentJoints[traversalIndex] = jointToParent;
    this->parents[traversalIndex] = parentLink;

    return true;
}

unsigned int Traversal::getNrOfVisitedLinks() const
{
    return this->links.size();
}

Link* Traversal::getLink(unsigned int traversalIndex) const
{
    return this->links[traversalIndex];
}

IJoint* Traversal::getParentJoint(unsigned int traversalIndex) const
{
    return this->toParentJoints[traversalIndex];
}

Link* Traversal::getParentLink(unsigned int traversalIndex) const
{
    return this->parents[traversalIndex];
}







}
