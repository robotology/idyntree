/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/Model.h>

#include <cassert>
#include <sstream>

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

const Link* Traversal::getLink(const TraversalIndex traversalIndex) const
{
    return this->links[traversalIndex];
}

const Link* Traversal::getBaseLink() const
{
    return this->getLink(0);
}

const IJoint* Traversal::getParentJoint(const TraversalIndex traversalIndex) const
{
    return this->toParentJoints[traversalIndex];
}

const Link* Traversal::getParentLink(const TraversalIndex traversalIndex) const
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

TraversalIndex Traversal::getTraversalIndexFromLinkIndex(const LinkIndex linkIndex) const
{
    return linkIndexToTraversalIndex[linkIndex];
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

LinkIndex Traversal::getChildLinkIndexFromJointIndex(const Model& model, const JointIndex jntIdx) const
{
    // Get the two links connected by this joint
    IJointConstPtr jnt = model.getJoint(jntIdx);
    if (!jnt)
    {
        return LINK_INVALID_INDEX;
    }

    LinkIndex link1 = jnt->getFirstAttachedLink();
    LinkIndex link2 = jnt->getSecondAttachedLink();

    // Get the traversal index of the child link (according to the traversal)
    LinkIndex childLink;
    if( this->isParentOf(link1,link2) )
    {
        childLink = link2;
    }
    else
    {
        childLink = link1;
    }

    return childLink;
}

LinkIndex Traversal::getParentLinkIndexFromJointIndex(const Model& model, const JointIndex jntIdx) const
{
    // Get the two links connected by this joint
    IJointConstPtr jnt = model.getJoint(jntIdx);
    if (!jnt)
    {
        return LINK_INVALID_INDEX;
    }

    LinkIndex link1 = jnt->getFirstAttachedLink();
    LinkIndex link2 = jnt->getSecondAttachedLink();

    // Get the traversal index of the child link (according to the traversal)
    LinkIndex parentLink;
    if( this->isParentOf(link1,link2) )
    {
        parentLink = link1;
    }
    else
    {
        parentLink = link2;
    }

    return parentLink;
}


std::string Traversal::toString(const Model & model) const
{
    std::stringstream ss;

    ss << "Traversal: " << std::endl;
    for(size_t i=0; i < this->getNrOfVisitedLinks(); i++ )
    {
        ss << "[" << i << "]\tLink: " << model.getLinkName(this->getLink(i)->getIndex()) << "[" << this->getLink(i)->getIndex() << "]" << std::endl;
        if( i > 0 )
        {
           ss << "\tJoint to parent : " << model.getJointName(this->getParentJoint(i)->getIndex()) << "[" << this->getParentJoint(i)->getIndex() << "]" << std::endl;
           ss << "\tParent link     : " << model.getLinkName(this->getParentLink(i)->getIndex()) << "[" << this->getParentLink(i)->getIndex() << "]" << std::endl;
        }
    }

    return ss.str();
}


}
