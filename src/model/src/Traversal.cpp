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
    reset(0);
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
    reset(0);
}

bool Traversal::reset(const unsigned int nrOfLinksInModel)
{
    this->links.resize(0);
    this->parents.resize(0);
    this->toParentJoints.resize(0);
    this->linkIndexToTraversalIndex.resize(nrOfLinksInModel,-1);

    return true;
}

bool Traversal::addTraversalElement(const Link* link, const IJoint* jointToParent, const Link* parentLink)
{
    // The traversal index of the link just added will be the current getNrOfVisitedLinks()
    this->linkIndexToTraversalIndex[link->getIndex()] = this->getNrOfVisitedLinks();

    this->links.push_back(link);
    this->toParentJoints.push_back(jointToParent);
    this->parents.push_back(parentLink);

    return true;
}


bool Traversal::addTraversalBase(const Link* link)
{
    if( this->getNrOfVisitedLinks() != 0 )
    {
        std::cerr << "[ERROR]  Traversal::addTraversalBase error :"
                  << " adding a base to Traversal that already has one." << std::endl;
        return false;
    }

    return addTraversalElement(link,NULL,NULL);
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

bool Traversal::reset(const Model& model)
{
    return reset(model.getNrOfLinks());
}

bool Traversal::isParentOf(const LinkIndex parentCandidate,
                           const LinkIndex childCandidate) const
{
    assert(childCandidate  < this->linkIndexToTraversalIndex.size());
    assert(parentCandidate < this->linkIndexToTraversalIndex.size());
    assert(childCandidate  >= 0);
    assert(parentCandidate >= 0);

    // If the childCandidate is not in the traversal, then clearly
    // the parentCandidate is not its parent for this traversal
    if( linkIndexToTraversalIndex[childCandidate] == -1 )
    {
        return false;
    }

    LinkConstPtr parentPtr = this->getParentLinkFromLinkIndex(childCandidate);
    if( parentPtr == 0 )
    {
        return false;
    }
    else
    {
        return (this->getParentLinkFromLinkIndex(childCandidate)->getIndex() == parentCandidate);
    }
}


}
