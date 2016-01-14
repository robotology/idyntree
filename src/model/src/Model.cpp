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

Model::Model(): defaultBaseLink(LINK_INVALID_INDEX), nrOfPosCoords(0), nrOfDOFs(0)
{

}

void Model::copy(const Model& other)
{
    // reset the base link, the real one will be copied later
    this->defaultBaseLink = LINK_INVALID_INDEX;

    // Add all the links, preserving the numbering
    for(unsigned int lnk=0; lnk < other.getNrOfLinks(); lnk++ )
    {
        this->addLink(other.linkNames[lnk],other.links[lnk]);
    }

    // Add all joints, preserving the numbering
    // reset the nrOfDOFs (it will be then update in addJoint)
    nrOfPosCoords = 0;
    nrOfDOFs = 0;

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
    nrOfPosCoords = 0;
    nrOfDOFs = 0;
    jointNames.resize(0);
    additionalFrames.resize(0);
    additionalFramesLinks.resize(0);
    frameNames.resize(0);
    neighbors.resize(0);
}

Model::~Model()
{
    destroy();
}

size_t Model::getNrOfLinks() const
{
    return links.size();
}

LinkIndex Model::getLinkIndex(const std::string& linkName) const
{
    for(size_t i=0; i < this->getNrOfLinks(); i++ )
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
    if( linkIndex >= 0 && linkIndex < (LinkIndex) this->getNrOfLinks() )
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

size_t Model::getNrOfJoints() const
{
    return joints.size();
}

JointIndex Model::getJointIndex(const std::string& jointName) const
{
    for(size_t i=0; i < this->getNrOfJoints(); i++ )
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
    if( jointIndex >= 0 && jointIndex < (JointIndex)this->getNrOfJoints() )
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
    // Check that the name is not already used by a link or frame
    if(isFrameNameUsed(name))
    {
        std::string error = "a link or frame of name " + name + " is already present in the model";
        reportError("Model","addLink",error.c_str());
        return LINK_INVALID_INDEX;
    }

    // Add the link to the vector of names and of links
    assert(links.size() == linkNames.size());
    linkNames.push_back(name);
    links.push_back(link);

    // add an empty adjacency list to the neighbors members
    neighbors.push_back(std::vector<Neighbor>());

    LinkIndex newLinkIndex = (LinkIndex)(links.size()-1);

    links[newLinkIndex].setIndex(newLinkIndex);

    // if this is the first link added to the model
    // and the defaultBaseLink has not been setted,
    // set the defaultBaseLink to be this link
    if( newLinkIndex == 0 && getDefaultBaseLink() == LINK_INVALID_INDEX )
    {
        setDefaultBaseLink(newLinkIndex);
    }

    return newLinkIndex;
}

bool Model::isJointNameUsed(const std::string jointName)
{
    return (JOINT_INVALID_INDEX != getJointIndex(jointName));
}


JointIndex Model::addJoint(const std::string& jointName, IJointConstPtr joint)
{
    assert(joint->getFirstAttachedLink() != joint->getSecondAttachedLink());

    if(isJointNameUsed(jointName))
    {
        std::string error = "a joint of name " + jointName + " is already present in the model";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Check that the joint is referring to links that are in the model
    LinkIndex firstLink = joint->getFirstAttachedLink();
    LinkIndex secondLink = joint->getSecondAttachedLink();
    if( firstLink < 0 || firstLink >= (LinkIndex)this->getNrOfLinks() ||
        secondLink < 0 || secondLink >= (LinkIndex)this->getNrOfLinks() )
    {
        std::string error = "joint " + jointName + " is attached to a link that does not exist";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Check that the joint is not connecting a link to itself
    if( firstLink == secondLink )
    {
        std::string error = "joint " + jointName + " is connecting link " + this->getLinkName(firstLink) + " to itself";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Update the joints and jointNames structure
    jointNames.push_back(jointName);
    joints.push_back(joint->clone());

    JointIndex thisJointIndex = (JointIndex)(joints.size()-1);

    // Update the adjacency list
    Neighbor firstLinkNeighbor;
    firstLinkNeighbor.neighborLink = secondLink;
    firstLinkNeighbor.neighborJoint = thisJointIndex;
    this->neighbors[firstLink].push_back(firstLinkNeighbor);

    Neighbor secondLinkNeighbor;
    secondLinkNeighbor.neighborLink = firstLink;
    secondLinkNeighbor.neighborJoint = thisJointIndex;
    this->neighbors[secondLink].push_back(secondLinkNeighbor);

    // Set the joint index and dof offset
    this->joints[thisJointIndex]->setIndex(thisJointIndex);
    this->joints[thisJointIndex]->setPosCoordsOffset(this->nrOfPosCoords);
    this->joints[thisJointIndex]->setDOFsOffset(this->nrOfDOFs);

    // Update the number of dofs
    this->nrOfPosCoords += this->joints[thisJointIndex]->getNrOfPosCoords();
    this->nrOfDOFs += this->joints[thisJointIndex]->getNrOfDOFs();

    return thisJointIndex;
}

size_t Model::getNrOfPosCoords() const
{
    return nrOfDOFs;
}

size_t Model::getNrOfDOFs() const
{
    return nrOfDOFs;
}


/////////////////////////////////////////////////////////////////////
///// Frame Related functions
/////////////////////////////////////////////////////////////////////
size_t Model::getNrOfFrames() const
{
    return (size_t)getNrOfLinks() + this->additionalFrames.size();
}

std::string Model::getFrameName(const FrameIndex frameIndex) const
{
    if( frameIndex >= 0 && frameIndex < (FrameIndex)this->getNrOfLinks() )
    {
        return linkNames[frameIndex];
    }
    else if( frameIndex >= (FrameIndex)this->getNrOfLinks() && frameIndex < (FrameIndex)this->getNrOfFrames() )
    {
        return frameNames[frameIndex-getNrOfLinks()];
    }
    else
    {
        return FRAME_INVALID_NAME;
    }
}

FrameIndex Model::getFrameIndex(const std::string& frameName) const
{
    for(size_t i=0; i < this->getNrOfLinks(); i++ )
    {
        if( frameName == linkNames[i] )
        {
            return (FrameIndex)i;
        }
    }

    for(size_t i=this->getNrOfLinks(); i < this->getNrOfFrames(); i++ )
    {
        if( frameName == this->frameNames[i-getNrOfLinks()] )
        {
            return (FrameIndex)i;
        }
    }

    return FRAME_INVALID_INDEX;
}

bool Model::isFrameNameUsed(const std::string frameName)
{
    return (FRAME_INVALID_INDEX != getFrameIndex(frameName));
}


bool Model::addAdditionalFrameToLink(const std::string& linkName,
                                     const std::string& frameName,
                                     Transform link_H_frame)
{
    // Check that the link actually exist
    LinkIndex linkIndex = this->getLinkIndex(linkName);
    if( linkIndex == LINK_INVALID_INDEX )
    {
        std::string error = "error adding frame " + frameName + " : a link of name "
                            + linkName + " is not present in the model";
        reportError("Model","addAdditionalFrameToLink",error.c_str());
        return false;
    }

    // Check that the frame name is not already used by a link or frame
    if(isFrameNameUsed(frameName))
    {
        std::string error = "a link or frame of name " + frameName + " is already present in the model";
        reportError("Model","addAdditionalFrameToLink",error.c_str());
        return false;
    }

    // If all error has passed, actually add the frame
    this->additionalFrames.push_back(link_H_frame);
    this->additionalFramesLinks.push_back(linkIndex);
    this->frameNames.push_back(frameName);

    return true;
}

Transform Model::getFrameTransform(const FrameIndex frameIndex) const
{
    // The link_H_frame transform for the link
    // main frame is the identity
    if( frameIndex < (FrameIndex) this->getNrOfLinks() ||
        frameIndex >= (FrameIndex) this->getNrOfFrames() )
    {
        return Transform::Identity();
    }

    // For an additonal frame the transform is instead stored
    // in the additionalFrames vector
    return this->additionalFrames[frameIndex-getNrOfLinks()];
}


LinkIndex Model::getFrameLink(const FrameIndex frameIndex) const
{
    // The link_H_frame transform for the link
    // main frame is the link it self
    if( frameIndex >= 0 &&
        frameIndex < (FrameIndex) this->getNrOfLinks() )
    {
        return (LinkIndex)frameIndex;
    }

    if( frameIndex >= (FrameIndex) this->getNrOfLinks() &&
        frameIndex < (FrameIndex) this->getNrOfFrames() )
    {
        // For an additonal frame the link index is instead stored
        // in the additionalFramesLinks vector
        return this->additionalFramesLinks[frameIndex-getNrOfLinks()];
    }

    // If the frameIndex is out of bounds, return an invalid index
    return LINK_INVALID_INDEX;
}






unsigned int Model::getNrOfNeighbors(const LinkIndex link) const
{
    assert(link < this->neighbors.size());
    return this->neighbors[link].size();
}

Neighbor Model::getNeighbor(const LinkIndex link, unsigned int neighborIndex) const
{
    assert(link < this->getNrOfLinks());
    assert(neighborIndex < this->getNrOfNeighbors(link));
    return this->neighbors[link][neighborIndex];
}

bool Model::setDefaultBaseLink(const LinkIndex linkIndex)
{
    if( linkIndex < 0 || linkIndex >= (LinkIndex) this->getNrOfLinks() )
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

bool Model::computeFullTreeTraversal(Traversal & traversal) const
{
    return computeFullTreeTraversal(traversal,this->getDefaultBaseLink());
}

struct stackEl { LinkConstPtr link; LinkConstPtr parent;};

void  addBaseLinkToTraversal(const Model & model, Traversal & traversal,
                                    LinkIndex linkToAdd, std::deque<stackEl> & linkToVisit)
{
    traversal.addTraversalBase(model.getLink(linkToAdd));

    stackEl el;
    el.link = model.getLink(linkToAdd);
    el.parent = 0;

    linkToVisit.push_back(el);
}

void addLinkToTraversal(const Model & model, Traversal & traversal,
                        LinkIndex linkToAdd, JointIndex parentJointToAdd, LinkIndex parentLinkToAdd,
                        std::deque<stackEl> & linkToVisit)
{
    traversal.addTraversalElement(model.getLink(linkToAdd),
                                  model.getJoint(parentJointToAdd),
                                  model.getLink(parentLinkToAdd));

    stackEl el;
    el.link = model.getLink(linkToAdd);
    el.parent = model.getLink(parentLinkToAdd);

    linkToVisit.push_back(el);
}

bool Model::computeFullTreeTraversal(Traversal & traversal, const LinkIndex traversalBase) const
{
    if( traversalBase < 0 || traversalBase >= (LinkIndex)this->getNrOfLinks() )
    {
        reportError("Model","computeFullTreeTraversal","requested traversalBase is out of bounds");
        return false;
    }

    // Resetting the traversal for populating it
    traversal.reset(*this);

    // A link is considered visit when all its child (given the traversalBase)
    // have been added to the traversal
    std::deque<stackEl> linkToVisit;

    // We add as first link the requested traversalBase
    addBaseLinkToTraversal(*this,traversal,traversalBase,linkToVisit);

    // while there is some link still to visit
    while( linkToVisit.size() > 0 )
    {
        assert(linkToVisit.size() <= this->getNrOfLinks());

        // DPS : we use linkToVisit as a stack
        LinkConstPtr visitedLink = linkToVisit.back().link;
        LinkConstPtr visitedLinkParent = linkToVisit.back().parent;
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkToVisit.pop_back();

        for(unsigned int neigh_i=0; neigh_i < this->getNrOfNeighbors(visitedLinkIndex); neigh_i++ )
        {
            // add to the stack all the neighbors, except for parent link
            // (if the visited link is the base one, add all the neighbors)
            // the visited link is already in the Traversal, so we can use it
            // to check for its parent
            Neighbor neighb = this->getNeighbor(visitedLinkIndex,neigh_i);
            if( visitedLinkParent == 0 || neighb.neighborLink != visitedLinkParent->getIndex() )
            {
                addLinkToTraversal(*this,traversal,neighb.neighborLink,
                    neighb.neighborJoint,visitedLink->getIndex(),linkToVisit);
            }
        }
    }

    // At this point the traversal should contain all the links
    // of the model
    assert(traversal.getNrOfVisitedLinks() == this->getNrOfLinks());

    return true;
}

std::string Model::toString() const
{
    std::stringstream ss;

    ss << "Model: " << std::endl;
    ss << "  Links: " << std::endl;
    for(size_t i=0; i < this->getNrOfLinks(); i++ )
    {
        ss << "    [" << i << "] " << this->getLinkName(i) << std::endl;
    }

    ss << "  Frames: " << std::endl;
    for(size_t i=this->getNrOfLinks(); i < this->getNrOfFrames(); i++ )
    {
        ss << "    [" << i << "] "
           << this->getFrameName(i)
           << " --> " << this->getLinkName(this->getFrameLink(i)) << std::endl;
    }

    ss << "  Joints: " << std::endl;
    for(size_t i=0; i < this->getNrOfJoints(); i++ )
    {
        ss << "    [" << i << "] "
           << this->getJointName(i) << " (dofs: " << this->getJoint(i)->getNrOfDOFs() << ") : "
           << this->getLinkName(this->getJoint(i)->getFirstAttachedLink()) << "<-->" << this->getLinkName(this->getJoint(i)->getSecondAttachedLink()) << std::endl;
    }

    return ss.str();
}





}
