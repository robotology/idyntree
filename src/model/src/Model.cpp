/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <deque>
#include <sstream>

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

    // Add all additional frames, preserving the numbering
    for(unsigned int addFrm=other.getNrOfLinks(); addFrm < other.getNrOfFrames(); addFrm++ )
    {
        std::string linkName = other.getLinkName(other.getFrameLink(addFrm));
        std::string frameName = other.getFrameName(addFrm);
        Transform link_H_frame = other.getFrameTransform(addFrm);
        this->addAdditionalFrameToLink(linkName,frameName,link_H_frame);
    }

    // Copy the default base link
    this->setDefaultBaseLink(other.getDefaultBaseLink());

    // Copy the solid shapes
    this->m_collisionSolidShapes = other.m_collisionSolidShapes;
    this->m_visualSolidShapes    = other.m_visualSolidShapes;
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

Model Model::copy() const
{
    return Model(*this);
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

    // Report an error and return an invalid index
    std::string error = "Impossible to find link " + linkName + " in the Model";
    reportError("Model","getLinkIndex",error.c_str());

    return LINK_INVALID_INDEX;
}

double Model::getTotalMass() const
{
    double totalMass = 0.0;

    for(size_t l=0; l < this->getNrOfLinks(); l++)
    {
        totalMass += this->getLink(l)->getInertia().getMass();
    }

    return totalMass;
}

bool Model::isValidLinkIndex(const LinkIndex index) const
{
    return (index != LINK_INVALID_INDEX) &&
           (index >= 0) && (index < this->getNrOfLinks());
}

std::string Model::getLinkName(const LinkIndex linkIndex) const
{
    if( linkIndex >= 0 && linkIndex < (LinkIndex) this->getNrOfLinks() )
    {
        return linkNames[linkIndex];
    }
    else
    {
        std::stringstream ss;
        ss << "LinkIndex " << linkIndex << " is not valid, should be between 0 and " << this->getNrOfLinks()-1;
        reportError("Model","getLinkName",ss.str().c_str());
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

    std::stringstream ss;
    ss << "jointName " << jointName << " not found in the model.";
    reportError("Model","getJointIndex",ss.str().c_str());

    return JOINT_INVALID_INDEX;
}

bool Model::isValidJointIndex(const JointIndex index) const
{
    return (index != JOINT_INVALID_INDEX) &&
           (index >= 0) && (index < this->getNrOfJoints());
}


std::string Model::getJointName(const JointIndex jointIndex) const
{
    if( jointIndex >= 0 && jointIndex < (JointIndex)this->getNrOfJoints() )
    {
        return jointNames[jointIndex];
    }
    else
    {
        std::stringstream ss;
        ss << "jointIndex " << jointIndex << " is not valid, should be between 0 and " << this->getNrOfJoints()-1;
        reportError("Model","getJointName",ss.str().c_str());
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

bool Model::isLinkNameUsed(const std::string linkName) const
{
    for(size_t i=0; i < this->getNrOfLinks(); i++ )
    {
        if( linkName == linkNames[i] )
        {
            return true;
        }
    }

    return false;
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

    // Add an empty vector of collision shapes
    m_collisionSolidShapes.getLinkSolidShapes().push_back(std::vector<SolidShape*>(0));

    // Add an empty vector of visual shapes
    m_visualSolidShapes.getLinkSolidShapes().push_back(std::vector<SolidShape*>(0));


    return newLinkIndex;
}

bool Model::isJointNameUsed(const std::string jointName) const
{
    for(size_t i=0; i < this->getNrOfJoints(); i++ )
    {
        if( jointName == jointNames[i] )
        {
            return true;
        }
    }

    return false;
}

JointIndex Model::addJoint(const std::string & link1, const std::string & link2,
                            const std::string & jointName, IJointConstPtr joint)
{
    IJointPtr jointCopy = joint->clone();

    LinkIndex link1Index = this->getLinkIndex(link1);
    LinkIndex link2Index = this->getLinkIndex(link2);

    if( link1Index == LINK_INVALID_INDEX )
    {
        std::string error = "a link of name " + link1 + " is not present in the model";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    if( link2Index == LINK_INVALID_INDEX )
    {
        std::string error = "a link of name " + link2 + " is not present in the model";
        reportError("Model","addJoint",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    jointCopy->setAttachedLinks(link1Index,link2Index);

    JointIndex jntIdx = addJoint(jointName,jointCopy);

    delete jointCopy;

    return jntIdx;
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

JointIndex Model::addJointAndLink(const std::string& existingLink,
                                  const std::string& jointName, IJointConstPtr joint,
                                  const std::string& newLinkName, Link& newLink)
{
    if( !(this->isLinkNameUsed(existingLink)) )
    {
        std::string error = "a link of name " + existingLink + " is not present in the model";
        reportError("Model","addJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    LinkIndex newAddedLink = this->addLink(newLinkName,newLink);

    if( newAddedLink == LINK_INVALID_INDEX )
    {
        std::string error = "Error adding link of name " + newLinkName;
        reportError("Model","addJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    return this->addJoint(existingLink,newLinkName,
                          jointName,joint);
}

JointIndex Model::insertLinkToExistingJointAndAddJointForDisplacedLink(const std::string & existingJointName,
                                     const std::string & unmovableLink,
                                     const Transform& _unmovableLink_X_newLink,
                                  const std::string& jointName, IJointConstPtr joint,
                                  const std::string& newLinkName, Link& newLink)
{


    // check the unmovableLink exists
    if( !(this->isLinkNameUsed(unmovableLink)) )
    {
        std::string error = "a link of name " + unmovableLink + " is not present in the model";
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    LinkIndex newAddedLink = this->addLink(newLinkName,newLink);

    if( newAddedLink == LINK_INVALID_INDEX )
    {
        std::string error = "Error adding link of name " + newLinkName;
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // check joint exists
    if( !(this->isJointNameUsed(existingJointName)) )
    {
        std::string error = "a joint of name " + existingJointName + " is not present in the model";
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    JointIndex existingJointIndex=this->getJointIndex(existingJointName);
    IJointPtr existingJoint=this->getJoint(existingJointIndex);

    // get current link indexes of the links attached to existing joint
    LinkIndex unmovableLinkIndex=this->getLinkIndex(unmovableLink);
    int LinkPositionInJoint=0;

    LinkIndex displacedLinkIndex;

    if (existingJoint->getFirstAttachedLink() == unmovableLinkIndex )
    {
        displacedLinkIndex=existingJoint->getSecondAttachedLink();
        LinkPositionInJoint=1;
    }

    if (existingJoint->getSecondAttachedLink() == unmovableLinkIndex )
    {
        displacedLinkIndex=existingJoint->getFirstAttachedLink();
        LinkPositionInJoint=2;
    }

    // check if the unmovable link is indeed attached to the joint
    if (LinkPositionInJoint == 0 ){
        std::string error = "a link of name " + unmovableLink + " is not attached to joint of name " + existingJointName + " in the model";
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // edit connections of existing link
    if (LinkPositionInJoint==1){
        existingJoint->setAttachedLinks(unmovableLinkIndex,newAddedLink);
        existingJoint->setRestTransform(_unmovableLink_X_newLink);
    }
    else
    {
        existingJoint->setAttachedLinks(newAddedLink,unmovableLinkIndex);
        existingJoint->setRestTransform(_unmovableLink_X_newLink.inverse());
    }

    // Check neighbors connections to find the previous connection index
    int previousConnectionIndex=-1;
    for(int i=0;i<this->neighbors[unmovableLinkIndex].size();i++ )
    {
        if ( this->neighbors[unmovableLinkIndex].at(i).neighborLink == displacedLinkIndex )
        {
            previousConnectionIndex=i;
        }
    }
    if (previousConnectionIndex == -1)
    {
        std::string error = "could not find neighbor connection from " + unmovableLink + " to " + this->getJointName(displacedLinkIndex) + " in the model";
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }

    // Update the adjacency list
    Neighbor unmovableLinkNeighbor;
    unmovableLinkNeighbor.neighborLink = newAddedLink;
    unmovableLinkNeighbor.neighborJoint = existingJointIndex;
    this->neighbors[unmovableLinkIndex].at(previousConnectionIndex)=unmovableLinkNeighbor;

    Neighbor newAddedLinkNeighbor;
    newAddedLinkNeighbor.neighborLink = unmovableLinkIndex;
    newAddedLinkNeighbor.neighborJoint = existingJointIndex;
    this->neighbors[newAddedLink].push_back(newAddedLinkNeighbor);

    // Erase neighbor connection from movable link to unmovable link
    previousConnectionIndex=-1;
    for(int i=0;i<this->neighbors[displacedLinkIndex].size();i++ )
    {
        if ( this->neighbors[displacedLinkIndex].at(i).neighborLink == unmovableLinkIndex )
        {
            previousConnectionIndex=i;
        }
    }
    if (previousConnectionIndex == -1)
    {
        std::string error = "could not find neighbor connection from " + this->getJointName(displacedLinkIndex) + " to " + unmovableLink + " in the model";
        reportError("Model","insertJointAndLink",error.c_str());
        return JOINT_INVALID_INDEX;
    }
    this->neighbors[displacedLinkIndex].erase(neighbors[displacedLinkIndex].begin()+previousConnectionIndex);
    const std::string movableLinkName=this->getLinkName(displacedLinkIndex);
    // we assume the rest transform for the new joint was set as newLink_x_movableLink
    return this->addJoint(newLinkName,movableLinkName,
                          jointName,joint);
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
        std::stringstream ss;
        ss << "frameIndex " << frameIndex << " is not valid, should be between 0 and " << this->getNrOfFrames()-1;
        reportError("Model","getFrameName",ss.str().c_str());
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

    std::stringstream ss;
    ss << "Frame named " << frameName << " not found in the model.";
    reportError("Model","getFrameIndex",ss.str().c_str());
    return FRAME_INVALID_INDEX;
}

bool Model::isValidFrameIndex(const FrameIndex index) const
{
    return (index != FRAME_INVALID_INDEX) &&
           (index >= 0) && (index < this->getNrOfFrames());
}

bool Model::isFrameNameUsed(const std::string frameName) const
{
    for(size_t i=0; i < this->getNrOfLinks(); i++ )
    {
        if( frameName == linkNames[i] )
        {
            return true;
        }
    }

    for(size_t i=this->getNrOfLinks(); i < this->getNrOfFrames(); i++ )
    {
        if( frameName == this->frameNames[i-getNrOfLinks()] )
        {
            return true;
        }
    }

    return false;
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
    assert(additionalFrames.size() == additionalFramesLinks.size());
    assert(additionalFrames.size() == frameNames.size());
    this->additionalFrames.push_back(link_H_frame);
    this->additionalFramesLinks.push_back(linkIndex);
    this->frameNames.push_back(frameName);

    return true;
}

Transform Model::getFrameTransform(const FrameIndex frameIndex) const
{
    // If frameIndex is invalid return an error
    if( frameIndex < 0 || frameIndex >= (FrameIndex) this->getNrOfFrames() )
    {
        std::stringstream ss;
        ss << "frameIndex " << frameIndex << " is not valid, should be between 0 and " << this->getNrOfFrames()-1;
        reportError("Model","getFrameTransform",ss.str().c_str());
        return Transform::Identity();
    }

    // The link_H_frame transform for the link
    // main frame is the identity
    if( frameIndex < (FrameIndex) this->getNrOfLinks() )
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
    std::stringstream ss;
    ss << "frameIndex " << frameIndex << " is not valid, should be between 0 and " << this->getNrOfFrames()-1;
    reportError("Model","getFrameLink",ss.str().c_str());
    return LINK_INVALID_INDEX;
}

bool Model::getLinkAdditionalFrames(const LinkIndex lnkIndex, std::vector<FrameIndex>& frameIndices) const
{
    if (!isValidLinkIndex(lnkIndex)) {
        std::stringstream ss;
        ss << "LinkIndex " << lnkIndex << " is not valid, should be between 0 and " << this->getNrOfLinks()-1;
        reportError("Model", "getLinkAdditionalFrames", ss.str().c_str());
        return false;
    }

    frameIndices.resize(0);
    // FrameIndex from 0 to this->getNrOfLinks()-1 are reserved for implicit frame of Links
    // with the corresponding LinkIndex, while the frameIndex from getNrOfLinks() to getNrOfFrames()-1
    // are the one of actual additional frames. See iDynTree::Model docs for more details
    for (FrameIndex frameIndex=this->getNrOfLinks(); frameIndex < this->getNrOfFrames(); frameIndex++) {
        if (this->getFrameLink(frameIndex) == lnkIndex) {
            frameIndices.push_back(frameIndex);
        }
    }

    return true;
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

bool Model::getInertialParameters(VectorDynSize& modelInertialParams) const
{
    // Resize vector if necessary
    if( modelInertialParams.size() != this->getNrOfLinks()*10 )
    {
        modelInertialParams.resize(10*this->getNrOfLinks());
    }

    for(LinkIndex linkIdx = 0; linkIdx < this->getNrOfLinks(); linkIdx++ )
    {
        Vector10       inertiaParamsBuf = links[linkIdx].inertia().asVector();

        toEigen(modelInertialParams).segment<10>(10*linkIdx) = toEigen(inertiaParamsBuf);
    }

    return true;
}


bool Model::updateInertialParameters(const VectorDynSize& modelInertialParams)
{
    if( modelInertialParams.size() != this->getNrOfLinks()*10 )
    {
        reportError("Model","updateInertialParameters","modelInertialParams has the wrong number of parameters");
        return false;
    }

    for(LinkIndex linkIdx = 0; linkIdx < this->getNrOfLinks(); linkIdx++ )
    {
        Vector10       inertiaParamsBuf;
        toEigen(inertiaParamsBuf) = toEigen(modelInertialParams).segment<10>(10*linkIdx);

        links[linkIdx].inertia().fromVector(inertiaParamsBuf);
    }

    return true;
}

ModelSolidShapes& Model::visualSolidShapes()
{
    return m_visualSolidShapes;
}

const ModelSolidShapes& Model::visualSolidShapes() const
{
    return m_visualSolidShapes;
}

ModelSolidShapes& Model::collisionSolidShapes()
{
    return m_collisionSolidShapes;
}

const ModelSolidShapes& Model::collisionSolidShapes() const
{
    return m_collisionSolidShapes;
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
