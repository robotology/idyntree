/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/SubModel.h>

#include <iDynTree/Model/IJoint.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

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
}

SubModelDecomposition::~SubModelDecomposition()
{
    this->setNrOfSubModels(0);
}

size_t SubModelDecomposition::getNrOfSubModels()
{
    return subModelTraversals.size();
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

size_t SubModelDecomposition::getSubModelOfLink(const LinkIndex& link)
{
    size_t ret = 0;

    if( link >= 0 &&  link < this->link2subModelIndex.size() )
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
    }

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



}
