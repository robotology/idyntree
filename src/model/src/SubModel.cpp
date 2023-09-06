// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SubModel.h>

#include <iDynTree/IJoint.h>
#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/JointState.h>

#include <cassert>
#include <algorithm>

namespace iDynTree
{

SubModelDecomposition::SubModelDecomposition()
{

}

SubModelDecomposition::SubModelDecomposition(const SubModelDecomposition& other)
{
    assert(false);
}

SubModelDecomposition& SubModelDecomposition::operator=(const SubModelDecomposition& other)
{
    assert(false);
    return *this;
}

SubModelDecomposition::~SubModelDecomposition()
{
    this->setNrOfSubModels(0);
}

size_t SubModelDecomposition::getNrOfSubModels() const
{
    return subModelTraversals.size();
}

size_t SubModelDecomposition::getNrOfLinks() const
{
    return link2subModelIndex.size();
}


void SubModelDecomposition::setNrOfSubModels(const size_t nrOfSubModels)
{
    for(size_t smIdx=0; smIdx < subModelTraversals.size(); smIdx++ )
    {
        delete subModelTraversals[smIdx];
        subModelTraversals[smIdx] = 0;
    }

    subModelTraversals.resize(nrOfSubModels);

    for(size_t smIdx=0; smIdx < subModelTraversals.size(); smIdx++ )
    {
        subModelTraversals[smIdx] = new Traversal();
    }
}

Traversal& SubModelDecomposition::getTraversal(const size_t subModelIndex)
{
    return *subModelTraversals[subModelIndex];
}

const Traversal& SubModelDecomposition::getTraversal(const size_t subModelIndex) const
{
    return *subModelTraversals[subModelIndex];
}

size_t SubModelDecomposition::getSubModelOfLink(const LinkIndex& link) const
{
    size_t ret = 0;

    if( link >= 0 &&  link < (LinkIndex) this->link2subModelIndex.size() )
    {
        ret = link2subModelIndex[link];
    }
    else
    {
        std::cerr << "SubModelDecomposition error: requested link index " << link
                  << "outside bounds " << std::endl;
    }

    return ret;
}

size_t SubModelDecomposition::getSubModelOfFrame(const Model & model, const FrameIndex& frame) const
{
    size_t ret = 0;
    LinkIndex linkOfFrame = model.getFrameLink(frame);

    if( linkOfFrame != LINK_INVALID_INDEX )
    {
        ret = this->getSubModelOfLink(linkOfFrame);
    }
    else
    {
        std::cerr << "SubModelDecomposition error: requested frame index " << frame
                  << "outside bounds " << std::endl;
    }

    return ret;
}


bool SubModelDecomposition::splitModelAlongJoints(const Model& model,
                                                  const Traversal& fullModelTraversal,
                                                  const std::vector< std::string >& splitJoints)
{
    // first we check that all the splitJoints are actually
    // joints of the model
    for(size_t jnt=0; jnt < splitJoints.size(); jnt++ )
    {
        if( model.getJointIndex(splitJoints[jnt]) == JOINT_INVALID_INDEX )
        {
            std::cerr << "[ERROR] SubModelDecomposition::splitModelAlongJoints error : "
                      << " requested to split the model along joint " << splitJoints[jnt]
                      << " but no joint with that is in the model. " << std::endl;
            return false;
        }

        // Check for duplicates (while it would be possible to handle duplicates,
        // duplicates are tipically associated with some error in the input parameters)
        // O(n^2) solution for checking the duplicates, it can be improved but this simple solution
        // should work fine for tipical uses cases
        for (size_t duplicateJnt=0; duplicateJnt < splitJoints.size(); duplicateJnt++ )
        {
            if (duplicateJnt != jnt && splitJoints[jnt] == splitJoints[duplicateJnt])
            {
                std::cerr << "[ERROR] SubModelDecomposition::splitModelAlongJoints error : "
                          << " the joint " << splitJoints[jnt] << " is both the element "
                          << jnt << " and " << duplicateJnt << " of the splitJoints list,"
                          << " please check the list of joints." << std::endl;
                return false;
            }
        }
    }

    // secondly we check for duplicates

    // The number of link in the decomposition is exactly
    // the number of links in the model
    this->link2subModelIndex.resize(model.getNrOfLinks());

    // we first need to resize the decomposition
    // the number of the submodels is exactly the number of the joints
    // aloing we are dividing the model, plus one
    this->setNrOfSubModels(splitJoints.size()+1);

    // we will assign links to the submodel using a
    // simple scheme: the base of the full tree traversal
    // is assinged to submodel 0. Then we visit the links
    // of the traversal, and we assign a link to the submodel
    // of its parent, unless it is connected to its parent
    // with a joint contained in splitJoints : in that case
    // the link is assigned to a new submodel, and it became
    // the base of the traversal of that submodel .
    size_t newSubModelIndexAvailableToUse = 0;

    for(size_t fullModelTraversalEl=0;
        fullModelTraversalEl < fullModelTraversal.getNrOfVisitedLinks();
        fullModelTraversalEl++)
    {
        LinkConstPtr visitedLink = fullModelTraversal.getLink(fullModelTraversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();

        // the link is assigned to a new subModel if
        // the visited link is the base or if it is connected
        // to its parent with a split joint
        bool isLinkBaseOfNewSubModel = false;
        bool isLinkBaseOfFullTreeTraversal = (fullModelTraversalEl == 0);

        if( isLinkBaseOfFullTreeTraversal )
        {
            isLinkBaseOfNewSubModel = true;
        }
        else
        {
            IJointConstPtr jointToParent = fullModelTraversal.getParentJoint(fullModelTraversalEl);
            std::string jointToParentName = model.getJointName(jointToParent->getIndex());

            bool isJointToParentASplitJoint =
                (std::find(splitJoints.begin(), splitJoints.end(), jointToParentName) != splitJoints.end());

            if( isJointToParentASplitJoint )
            {
                isLinkBaseOfNewSubModel = true;
            }

        }

        if( isLinkBaseOfNewSubModel )
        {
            this->link2subModelIndex[visitedLinkIndex] = newSubModelIndexAvailableToUse;
            this->subModelTraversals[newSubModelIndexAvailableToUse]->reset(model);
            this->subModelTraversals[newSubModelIndexAvailableToUse]->addTraversalBase(visitedLink);

            newSubModelIndexAvailableToUse++;
        }
        else
        {
            // In this case we know that the link has a parent,
            // otherwise it would be a base of the submodel traversal
            LinkConstPtr parentLink = fullModelTraversal.getParentLink(fullModelTraversalEl);
            IJointConstPtr jointToParent = fullModelTraversal.getParentJoint(fullModelTraversalEl);

            size_t subModelIndex = this->link2subModelIndex[visitedLinkIndex]
                                 = this->link2subModelIndex[parentLink->getIndex()];

            this->subModelTraversals[subModelIndex]->addTraversalElement(visitedLink,jointToParent,parentLink);
        }
    }

    return true;
}

void computeTransformToTraversalBase(const Model& fullModel,
                                     const Traversal& subModelTraversal,
                                     const JointPosDoubleArray& jointPos,
                                           LinkPositions& traversalBase_H_link)
{
    for(unsigned int traversalEl=0; traversalEl < subModelTraversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
        LinkConstPtr parentLink  = subModelTraversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = subModelTraversal.getParentJoint(traversalEl);

        // If this is the traversal base
        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent.
            // In this case the position of the base with respect to the base is simply
            // an identity transform
            traversalBase_H_link(visitedLink->getIndex()) = iDynTree::Transform::Identity();
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            traversalBase_H_link(visitedLink->getIndex()) =
                traversalBase_H_link(parentLink->getIndex())*
                    toParentJoint->getTransform(jointPos,parentLink->getIndex(),visitedLink->getIndex());
        }
    }

    return;
}

void computeTransformToSubModelBase(const Model& fullModel,
                                    const SubModelDecomposition& subModelDecomposition,
                                    const JointPosDoubleArray& jointPos,
                                          LinkPositions& subModelBase_H_link)
{
    for(size_t subModel = 0;
               subModel < subModelDecomposition.getNrOfSubModels();
               subModel++ )
    {
        computeTransformToTraversalBase(fullModel,
                                        subModelDecomposition.getTraversal(subModel),
                                        jointPos,
                                        subModelBase_H_link);
    }
}



}
