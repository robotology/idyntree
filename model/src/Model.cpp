/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <cassert>
#include <deque>

namespace iDynTree
{

Model::Model(): defaultBaseLink(0)
{

}

void Model::copy(const Model& other)
{
    // Add all the links, preserving the numbering
    for(unsigned int lnk=0; lnk < other.getNrOfLinks(); lnk++ )
    {
        this->addLink(other.linkNames[lnk],other.links[lnk]);
    }

    // Add all joints, preserving the numbering
    for(unsigned int jnt=0; jnt < other.getNrOfJoints(); jnt++ )
    {
        this->addJoint(other.jointNames[jnt],other.joints[jnt]);
    }

    // Copy the default base link
    this->setDefaultBaseLink(other.getDefaultBaseLink());
}


Model::Model(const Model& other)
{
    copy(other);
}

Model& Model::operator=(const Model& other)
{
    if( &other != this )
    {
        destroy();
        copy(other);
    }

    return *this;
}

void Model::destroy()
{
    links.resize(0);
    linkNames.resize(0);
    for(unsigned int jnt=0; jnt < this->getNrOfJoints(); jnt++ )
    {
        delete this->joints[jnt];
        this->joints[jnt] = 0;
    }
    joints.resize(0);
    jointNames.resize(0);
    neighbors.resize(0);
}

Model::~Model()
{
    destroy();
}

int Model::getNrOfLinks() const
{
    return links.size();
}

LinkIndex Model::getLinkIndex(const std::string& linkName) const
{
    for(int i=0; i < 0; i++ )
    {
        if( linkName == linkNames[i] )
        {
            return i;
        }
    }

    return LINK_INVALID_INDEX;
}

std::string Model::getLinkName(const LinkIndex linkIndex) const
{
    if( linkIndex >= 0 && linkIndex < this->getNrOfLinks() )
    {
        return linkNames[linkIndex];
    }
    else
    {
        return LINK_INVALID_NAME;
    }
}

Link* Model::getLink(const LinkIndex linkIndex)
{
    return &(links[linkIndex]);
}

const Link* Model::getLink(const LinkIndex linkIndex) const
{
    return &(links[linkIndex]);
}

int Model::getNrOfJoints() const
{
    return joints.size();
}

LinkIndex Model::getJointIndex(const std::string& jointName) const
{
    for(int i=0; i < 0; i++ )
    {
        if( jointName == jointNames[i] )
        {
            return i;
        }
    }

    return JOINT_INVALID_INDEX;
}

std::string Model::getJointName(const JointIndex jointIndex) const
{
    if( jointIndex >= 0 && jointIndex < this->getNrOfJoints() )
    {
        return jointNames[jointIndex];
    }
    else
    {
        return JOINT_INVALID_NAME;
    }
}

IJoint* Model::getJoint(const JointIndex jointIndex)
{
    return (joints[jointIndex]);
}

const IJoint* Model::getJoint(const JointIndex jointIndex) const
{
    return (joints[jointIndex]);
}

bool Model::isLinkNameUsed(const std::string linkName)
{
    return (LINK_INVALID_INDEX != getLinkIndex(linkName));
}

LinkIndex Model::addLink(const std::string& name, const Link& link)
{
    // Check that the name is not already used
    if(isLinkNameUsed(name))
    {
        std::string error = "a link of name " + name + " is already present in the model";
        reportError("Model","addLink",error.c_str());
        return LINK_INVALID_INDEX;
    }

    // Add the link to the vector of names and of links
    assert(links.size() == linkNames.size());
    linkNames.push_back(name);
    links.push_back(link);

    // add an empty adjacency list to the neighbors members
    neighbors.push_back(std::vector<Neighbor>());

    return (LinkIndex)(links.size()-1);
}

bool Model::isJointNameUsed(const std::string jointName)
{
    return (JOINT_INVALID_INDEX != getJointIndex(jointName));
}


JointIndex Model::addJoint(const std::string& jointName, IJointConstPtr joint)
{
    if(isJointNameUsed(jointName))
    {
        std::string error = "a joint of name " + jointName + " is already present in the model";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Check that the joint is referring to links that are in the model
    LinkIndex firstLink = joint->getFirstAttachedLink();
    LinkIndex secondLink = joint->getSecondAttachedLink();
    if( firstLink < 0 || firstLink >= this->getNrOfLinks() ||
        secondLink < 0 || secondLink >= this->getNrOfLinks() )
    {
        std::string error = "joint " + jointName + " is attached to a link that does not exist";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Check that the joint is not connecting a link to itself
    if( firstLink == secondLink )
    {
        std::string error = "joint " + jointName + " is connecting a link to itself";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Update the joints and jointNames structure
    jointNames.push_back(jointName);
    joints.push_back(joint->clone());

    JointIndex thisJointIndex = (JointIndex)(joints.size()-1);

    // Update the adjacency list
    Neighbor firstLinkNeighbor;
    firstLinkNeighbor.neighborLink = getLink(secondLink);
    firstLinkNeighbor.neighborJoint = getJoint(thisJointIndex);
    this->neighbors[firstLink].push_back(firstLinkNeighbor);

    Neighbor secondLinkNeighbor;
    secondLinkNeighbor.neighborLink = getLink(firstLink);
    secondLinkNeighbor.neighborJoint = getJoint(thisJointIndex);
    this->neighbors[secondLink].push_back(secondLinkNeighbor);

    return thisJointIndex;
}

unsigned int Model::getNrOfNeighbors(const LinkIndex link)
{
    return this->neighbors[link].size();
}


Neighbor Model::getNeighbor(const LinkIndex link, unsigned int neighborIndex)
{
    return this->neighbors[link][neighborIndex];
}

bool Model::setDefaultBaseLink(const LinkIndex linkIndex)
{
    if( linkIndex < 0 || linkIndex >= this->getNrOfLinks() )
    {
        return false;
    }
    else
    {
        defaultBaseLink = linkIndex;
        return true;
    }
}

LinkIndex Model::getDefaultBaseLink() const
{
    return defaultBaseLink;
}

bool Model::computeFullTreeTraversal(Traversal & traversal)
{
    return computeFullTreeTraversal(traversal,this->getDefaultBaseLink());
}

struct stackEl { LinkPtr link; LinkPtr parent;};

void addLinkToTraversal(Traversal & traversal, int & traversalFirstEmptySlot,
                        LinkPtr linkToAdd, IJointPtr parentJointToAdd, LinkPtr parentLinkToAdd,
                        std::deque<stackEl> & linkToVisit)
{
    traversal.setTraversalElement(traversalFirstEmptySlot,linkToAdd,parentJointToAdd,parentLinkToAdd);

    traversalFirstEmptySlot++;

    stackEl el;
    el.link = linkToAdd;
    el.parent = parentLinkToAdd;

    linkToVisit.push_back(el);
}

bool Model::computeFullTreeTraversal(Traversal & traversal, const LinkIndex traversalBase)
{
    if( traversalBase < 0 || traversalBase >= this->getNrOfLinks() )
    {
        reportError("Model","computeFullTreeTraversal","requested traversalBase is out of bounds");
        return false;
    }

    // The full tree traversal is a traversal spanning all the links
    // of a model, so it include getNrOfLinks() visited links
    traversal.reset(this->getNrOfLinks());

    // A link is considered visit when all its child (given the traversalBase)
    // have been added to the traversal
    std::deque<stackEl> linkToVisit;

    // We add as first link the requested traversalBase
    int traversalFirstEmptySlot = 0;
    addLinkToTraversal(traversal,traversalFirstEmptySlot,this->getLink(traversalBase),
                       (IJointPtr)0,(LinkPtr)0,linkToVisit);

    // while there is some link still to visit
    unsigned int visitNumber=0;
    while( linkToVisit.size() > 0 )
    {
        // DPS : we use linkToVisit as a stack
        LinkPtr visitedLink = linkToVisit.back().link;
        LinkPtr visitedLinkParent = linkToVisit.back().parent;
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkToVisit.pop_back();

        for(unsigned int neigh_i=0; neigh_i <= this->getNrOfNeighbors(visitedLinkIndex); neigh_i++ )
        {
            // add to the stack all the neighbors, except for parent
            // the visited link is already in the Traversal, so we can use it
            // to check the parent
            Neighbor neighb = this->getNeighbor(visitedLinkIndex,neigh_i);
            if( neighb.neighborLink != visitedLinkParent )
            {
                addLinkToTraversal(traversal,traversalFirstEmptySlot,neighb.neighborLink,
                    neighb.neighborJoint,visitedLink,linkToVisit);
            }
        }
    }

    return true;
}




}
