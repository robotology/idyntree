/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <cassert>
#include <set>


namespace iDynTree
{

/**
 * Check the condition for deciding if a model has a fake base link.
 * The three conditions for a base link to be considered "fake" are:
 *  * if the base link is massless,
 *  * if the base link has only one child,
 *  * if the base link is attached to its only child with a fixed joint,
 *
 */
bool isFakeLink(const Model& modelWithFakeLinks, const LinkIndex linkToCheck)
{
    // First condition: base link is massless
    double mass = modelWithFakeLinks.getLink(linkToCheck)->getInertia().getMass();
    if( mass > 0.0 )
    {
        return false;
    }

    // Second condition: the base link has only one child
    if( modelWithFakeLinks.getNrOfNeighbors(linkToCheck) != 1 )
    {
        return false;
    }

    // Third condition: the base link is attached to its child with a fixed joint
    Neighbor neigh = modelWithFakeLinks.getNeighbor(linkToCheck,0);
    if( modelWithFakeLinks.getJoint(neigh.neighborJoint)->getNrOfDOFs() > 0 )
    {
        return false;
    }

    return true;
}

bool removeFakeLinks(const Model& modelWithFakeLinks,
                     Model& modelWithoutFakeLinks)
{
    std::set<std::string> linkToRemove;
    std::set<std::string> jointToRemove;

    std::string newDefaultBaseLink = modelWithFakeLinks.getLinkName(modelWithFakeLinks.getDefaultBaseLink());

    // We iterate on all the links in the model
    // and check which one are "fake links", according
    // to our definition
    for(LinkIndex lnkIdx = 0; lnkIdx < modelWithFakeLinks.getNrOfLinks(); lnkIdx++ )
    {
        if( isFakeLink(modelWithFakeLinks,lnkIdx) )
        {
            linkToRemove.insert(modelWithFakeLinks.getLinkName(lnkIdx));
            JointIndex jntIdx = modelWithFakeLinks.getNeighbor(lnkIdx,0).neighborJoint;
            jointToRemove.insert(modelWithFakeLinks.getJointName(jntIdx));

            // if the fake link is the default base, we also need to update the
            // default base in the new model
            if( lnkIdx == modelWithFakeLinks.getDefaultBaseLink() )
            {
                LinkIndex newBaseIndex =  modelWithFakeLinks.getNeighbor(lnkIdx,0).neighborLink;
                newDefaultBaseLink = modelWithFakeLinks.getLinkName(newBaseIndex);
            }
        }
    }

    // First, we create the new model obtained
    // removing all the fake links (and relative joints)
    modelWithoutFakeLinks = Model();
    // Add all links, except for the one that we need to remove
    for(unsigned int lnk=0; lnk < modelWithFakeLinks.getNrOfLinks(); lnk++ )
    {
        std::string linkToAdd = modelWithFakeLinks.getLinkName(lnk);
        std::cout << "Considering link " << linkToAdd << std::endl;
        if( linkToRemove.find(linkToAdd) == linkToRemove.end() )
        {
            std::cout << "Adding link " << linkToAdd << std::endl;
            modelWithoutFakeLinks.addLink(linkToAdd,*modelWithFakeLinks.getLink(lnk));
        }
    }

    // Add all joints, preserving the serialization
    for(unsigned int jnt=0; jnt < modelWithFakeLinks.getNrOfJoints(); jnt++ )
    {
        std::string jointToAdd = modelWithFakeLinks.getJointName(jnt);
        if( jointToRemove.find(jointToAdd) == jointToRemove.end() )
        {
            // we need to change the link index in the new joints
            // to match the new link serialization
            IJointPtr newJoint = modelWithFakeLinks.getJoint(jnt)->clone();
            std::string firstLinkName = modelWithFakeLinks.getLinkName(newJoint->getFirstAttachedLink());
            std::string secondLinkName = modelWithFakeLinks.getLinkName(newJoint->getSecondAttachedLink());
            JointIndex  firstLinkNewIndex = modelWithoutFakeLinks.getLinkIndex(firstLinkName);
            JointIndex  secondLinkNewIndex = modelWithoutFakeLinks.getLinkIndex(secondLinkName);
            newJoint->setAttachedLinks(firstLinkNewIndex,secondLinkNewIndex);

            modelWithoutFakeLinks.addJoint(jointToAdd,newJoint);

            delete newJoint;
        }
    }

    // Then we add all frames (i.e. fake links that we removed from the model)
    for(unsigned int lnk=0; lnk < modelWithFakeLinks.getNrOfLinks(); lnk++ )
    {
        std::string fakeLinkName = modelWithFakeLinks.getLinkName(lnk);
        if( linkToRemove.find(fakeLinkName) != linkToRemove.end() )
        {
            LinkIndex fakeLinkOldIndex = modelWithFakeLinks.getLinkIndex(fakeLinkName);

            // One of the condition for a base to be fake is to
            // be connected to the real link with a fixed joint, so
            // their transform can be obtained without specifying the joint positions
            assert(modelWithFakeLinks.getNrOfNeighbors(fakeLinkOldIndex) == 1);

            JointIndex fakeLink_realLink_joint = modelWithFakeLinks.getNeighbor(fakeLinkOldIndex,0).neighborJoint;
            LinkIndex   realLinkOldIndex = modelWithFakeLinks.getNeighbor(fakeLinkOldIndex,0).neighborLink;
            std::string realLinkName = modelWithFakeLinks.getLinkName(realLinkOldIndex);

            // Get the transform
            iDynTree::Transform realLink_H_fakeLink =
                modelWithFakeLinks.getJoint(fakeLink_realLink_joint)->getRestTransform(realLinkOldIndex,fakeLinkOldIndex);

            // Add the fake base as a frame
            modelWithoutFakeLinks.addAdditionalFrameToLink(realLinkName,fakeLinkName,realLink_H_fakeLink);
        }
    }

    // Set the default base link
    return modelWithoutFakeLinks.setDefaultBaseLink(modelWithoutFakeLinks.getLinkIndex(newDefaultBaseLink));
}


}
