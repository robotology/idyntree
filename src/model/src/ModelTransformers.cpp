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
#include <iDynTree/Model/SubModel.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/PrismaticJoint.h>

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
    for(LinkIndex lnkIdx = 0; lnkIdx < (LinkIndex)modelWithFakeLinks.getNrOfLinks(); lnkIdx++ )
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
        if( linkToRemove.find(linkToAdd) == linkToRemove.end() )
        {
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

/**
 * Compute the CRBA for each link in a submodel, represented by
 * a given traversal.
 */
void computeCompositeRigidBodyInertiaSubModel(const Model& fullModel,
                                              const Traversal& subModelTraversal,
                                              const FreeFloatingPos& jointPos,
                                              LinkInertias& linkSubModelCRBs)
{
    /**
     * Forward pass: initialize the CRBI
     * of each link to its own inertia.
     */
    for(unsigned int traversalEl=0; traversalEl < subModelTraversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkSubModelCRBs(visitedLinkIndex) = visitedLink->getInertia();
    }

    /**
     * Backward pass: for each link compute the
     * CRB of the link (given the traversal).
     */
    for(int traversalEl = subModelTraversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = subModelTraversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = subModelTraversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = subModelTraversal.getParentJoint(traversalEl);

        // If the visited link is not the base one, add its
        // CRBI to the CRBI of the parent
        // \todo TODO streamline the check "is Link the floating base"
        // given a traversal
        if( parentLink )
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();

            linkSubModelCRBs(parentLinkIndex) = linkSubModelCRBs(parentLinkIndex) +
                (toParentJoint->getTransform(jointPos.jointPos(),parentLinkIndex,visitedLinkIndex))*linkSubModelCRBs(visitedLinkIndex);
        }
    }
}



/**
 * Given a model, build for each link a list of
 * all the additional frames attached to it.
 *
 */
void buildLinkToAdditionalFramesList(const Model& fullModel,
                                     std::vector< std::vector<FrameIndex> > & link2additionalFramesAdjacencyList)
{
    // Resize the data structure
    link2additionalFramesAdjacencyList.resize(fullModel.getNrOfLinks());

    // Iterate on all the frames and add them to the right link list
    for(FrameIndex additionalFrame = fullModel.getNrOfLinks();
        additionalFrame < (FrameIndex)fullModel.getNrOfFrames();
        additionalFrame++)
    {
        LinkIndex linkOfAdditionalFrame = fullModel.getFrameLink(additionalFrame);
        link2additionalFramesAdjacencyList[linkOfAdditionalFrame].push_back(additionalFrame);
    }
}

void reducedModelAddAdditionalFrames(const Model& fullModel,
                                           Model& reducedModel,
                                     const std::string linkInReducedModel,
                                     const Traversal& linkSubModel,
                                     const FreeFloatingPos& pos,
                                           LinkPositions& subModelBase_X_link)
{
    // First compute the transform between each link in the submodel and the submodel base
    computeTransformToTraversalBase(fullModel,linkSubModel,pos.jointPos(),subModelBase_X_link);

    // We then need to compute the list of additional frames for each link
    // This is a rather inefficient operation (given how frame information is stored in the Model
    // class) and is duplicated for each submodel, but this function should be called
    // just at configuration time, so this should be ok
    std::vector< std::vector<FrameIndex> > link2additionalFramesAdjacencyList;
    buildLinkToAdditionalFramesList(fullModel,link2additionalFramesAdjacencyList);

    // All the links in the traversal are lumped into the base
    // We then need to add all additional frames (and all the links
    // except for the base) to the model as additional frames of linkInReducedModel
    for(unsigned int traversalEl=0; traversalEl < linkSubModel.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = linkSubModel.getLink(traversalEl);
        LinkConstPtr parentLink  = linkSubModel.getParentLink(traversalEl);

        LinkIndex visitedLinkIndex = visitedLink->getIndex();

        // Add the link frame as an additional frame only for link that are not the submodel base
        if( parentLink != 0 )
        {
            std::string additionalFrameName = fullModel.getFrameName(visitedLinkIndex);

            Transform subModelBase_H_additionalFrame = subModelBase_X_link(visitedLinkIndex);


            reducedModel.addAdditionalFrameToLink(linkInReducedModel,additionalFrameName,
                                               subModelBase_H_additionalFrame);
        }

        // For all the link of the submodel, transfer their additional frame
        // to the lumped link in the reduced model
        for(size_t i = 0; i < link2additionalFramesAdjacencyList[visitedLinkIndex].size(); i++ )
        {
            FrameIndex additionalFrame = link2additionalFramesAdjacencyList[visitedLinkIndex][i];
            std::string additionalFrameName = fullModel.getFrameName(additionalFrame);

            Transform visitedLink_H_additionalFrame  = fullModel.getFrameTransform(additionalFrame);
            Transform subModelBase_H_visitedLink     = subModelBase_X_link(visitedLinkIndex);
            Transform subModelBase_H_additionalFrame =
                subModelBase_H_visitedLink*visitedLink_H_additionalFrame;

            reducedModel.addAdditionalFrameToLink(linkInReducedModel,additionalFrameName,
                                               subModelBase_H_additionalFrame);
        }
    }
}

void reducedModelAddSolidShapes(const Model& fullModel,
                                      Model& reducedModel,
                                const std::string linkInReducedModel,
                                const Traversal& linkSubModel,
                                const FreeFloatingPos& pos,
                                      LinkPositions& subModelBase_X_link)
{
    // First compute the transform between each link in the submodel and the submodel base
    computeTransformToTraversalBase(fullModel,linkSubModel,pos.jointPos(),subModelBase_X_link);

    LinkIndex subModelBaseIndexInReducedModel = reducedModel.getLinkIndex(linkInReducedModel);

    // All the geometries collected in the traversal are lumped into the base link
    for(unsigned int traversalEl=0; traversalEl < linkSubModel.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = linkSubModel.getLink(traversalEl);
        LinkConstPtr parentLink  = linkSubModel.getParentLink(traversalEl);

        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        Transform subModelBase_H_visitedLink     = subModelBase_X_link(visitedLinkIndex);

        // Add visual shapes to the new lumped link, i.e. the subModelBase
        auto& visualSolidShapes = fullModel.visualSolidShapes().getLinkSolidShapes();
        for(int shapeIdx=0; shapeIdx < visualSolidShapes[visitedLinkIndex].size(); shapeIdx++)
        {
            // Clone the shape
            SolidShape * copiedShape = visualSolidShapes[visitedLinkIndex][shapeIdx]->clone();

            // Update shape transform from the old link to the new link
            Transform visitedLink_H_shape = visualSolidShapes[visitedLinkIndex][shapeIdx]->getLink_H_geometry();
            copiedShape->setLink_H_geometry(subModelBase_H_visitedLink * visitedLink_H_shape);

            // Ownership of the new pointer is transfered to the ModelSolidShapes class
            // (it will be eventually deleted by the close() method
            reducedModel.visualSolidShapes().getLinkSolidShapes()[subModelBaseIndexInReducedModel].push_back(copiedShape);
        }

        // Add collision shapes to the new lumped link, i.e. the subModelBase
        auto& collisionSolidShapes = fullModel.collisionSolidShapes().getLinkSolidShapes();
        for(int shapeIdx=0; shapeIdx < collisionSolidShapes[visitedLinkIndex].size(); shapeIdx++)
        {
            // Clone the shape : ownership of the new pointer is transfered to the ModelSolidShapes class
            // (it will be eventually deleted by the close() method
            SolidShape * copiedShape = collisionSolidShapes[visitedLinkIndex][shapeIdx]->clone();

            // Update shape transform from the old link to the new link
            Transform visitedLink_H_shape = collisionSolidShapes[visitedLinkIndex][shapeIdx]->getLink_H_geometry();
            copiedShape->setLink_H_geometry(subModelBase_H_visitedLink*visitedLink_H_shape);

            // Ownership of the new pointer is transfered to the ModelSolidShapes class
            // (it will be eventually deleted by the close() method
            reducedModel.collisionSolidShapes().getLinkSolidShapes()[subModelBaseIndexInReducedModel].push_back(copiedShape);
        }
    }

}

bool createReducedModel(const Model& fullModel,
                        const std::vector< std::string >& jointsInReducedModel,
                        Model& reducedModel)
{
    // We use the default traversal for deciding the base links of the reduced model
    Traversal fullModelTraversal;
    fullModel.computeFullTreeTraversal(fullModelTraversal);

    // To compute the inertia and all the other properties of the links of
    // the reduced model, we need to build a SubModelDecomposition of the full
    // model, in which every SubModel corresponds to a link in the reduced model
    SubModelDecomposition subModels;

    bool ok = subModels.splitModelAlongJoints(fullModel,fullModelTraversal,jointsInReducedModel);

    if( !ok )
    {
         std::cerr << "[ERROR] createReducedModel error : "
                      << " error in splitting models across joints. "
                      << std::endl;
        return false;
    }

    // We then compute all the links of the reduced model : each link will
    // keep the name of the relative submodel
    size_t nrOfLinksInReducedModel = subModels.getNrOfSubModels();

    // We need the buffer to for each link the transform to its submodel base
    // and the composite rigid body inertia wrt to the submodel
    LinkInertias crbas(fullModel);
    LinkPositions subModelBase_X_link(fullModel);

    // The position for the joint removed from the model is supposed to be 0
    FreeFloatingPos jointPos(fullModel);
    // \todo used an appropriate method here
    for(size_t posCoord=0; posCoord < fullModel.getNrOfPosCoords(); posCoord++)
    {
        jointPos.jointPos()(posCoord) = 0.0;
    }

    for(size_t linkInReducedModel = 0;
               linkInReducedModel < nrOfLinksInReducedModel;
               linkInReducedModel++)
    {
        assert(linkInReducedModel < subModels.getNrOfSubModels());
        assert(subModels.getTraversal(linkInReducedModel).getNrOfVisitedLinks() > 0);
        LinkIndex linkFullModelIndex = subModels.getTraversal(linkInReducedModel).getLink(0)->getIndex();

        std::string linkName =
            fullModel.getLinkName(linkFullModelIndex);

        // Compute inertia with an appropriate loop
        computeCompositeRigidBodyInertiaSubModel(fullModel,subModels.getTraversal(linkInReducedModel),
                                                 jointPos,crbas);

        // The link inertia is just its own CRBA in the submodel
        iDynTree::SpatialInertia linkInertia = crbas(linkFullModelIndex);

        Link newLinkForReducedModel;
        newLinkForReducedModel.setInertia(linkInertia);

        // Add the new link to the reducedModel
        reducedModel.addLink(linkName,newLinkForReducedModel);

        // Get not-base links of the submodel and transform
        // them in additional frames in the reduced model
        // and get additional frames and copy them to reduced model
        reducedModelAddAdditionalFrames(fullModel,reducedModel,
                                        linkName,subModels.getTraversal(linkInReducedModel),
                                        jointPos,subModelBase_X_link);

        // Lump the visual and collision shapes in the new model
        reducedModelAddSolidShapes(fullModel,reducedModel,
                                   linkName,subModels.getTraversal(linkInReducedModel),
                                   jointPos,subModelBase_X_link);

    }

    // We then add to the model all the joints, that are exactly the
    // jointsInReducedModel. The child link of the joint will be the
    // same for sure, but the parent link could have changed for model
    // reduction: we need to account for that
    size_t nrOfJointsInReducedModel = jointsInReducedModel.size();

    for(size_t jointInReducedModel = 0;
               jointInReducedModel < nrOfJointsInReducedModel;
               jointInReducedModel++)
    {
        std::string jointName =
            jointsInReducedModel[jointInReducedModel];

        // We get the two links that the joint connected in the old
        // full model
        JointIndex fullModelJointIndex = fullModel.getJointIndex(jointName);

        IJointConstPtr oldJoint = fullModel.getJoint(fullModelJointIndex);
        LinkIndex oldLink1 = oldJoint->getFirstAttachedLink();
        LinkIndex oldLink2 = oldJoint->getSecondAttachedLink();

        // We get the new link that the joint connects, after the lumping
        // (consider that the subModel index not matches the indices of the new links
        // in the new reduced model
        LinkIndex newLink1 = (LinkIndex) subModels.getSubModelOfLink(oldLink1);
        LinkIndex newLink2 = (LinkIndex) subModels.getSubModelOfLink(oldLink2);

        Transform newLink1_X_oldLink1 = subModelBase_X_link(oldLink1);
        Transform newLink2_X_oldLink2 = subModelBase_X_link(oldLink2);

        // \todo TODO handle this in a joint-agnostic way,
        // possibly extending the joint interface
        IJointPtr newJoint = 0;
        if( dynamic_cast<const FixedJoint*>(oldJoint) )
        {
            const FixedJoint* oldJointFixed = dynamic_cast<const FixedJoint*>(oldJoint);

            Transform oldLink1_X_oldLink2 = oldJointFixed->getRestTransform(oldLink1,oldLink2);
            Transform newLink1_X_newLink2 = newLink1_X_oldLink1*oldLink1_X_oldLink2*newLink2_X_oldLink2.inverse();

            FixedJoint* newJointFixed = new FixedJoint(newLink1,newLink2,newLink1_X_newLink2);

            newJoint = (IJointPtr) newJointFixed;
        }
        else if( dynamic_cast<const RevoluteJoint*>(oldJoint) )
        {
            const RevoluteJoint* oldJointRevolute = dynamic_cast<const RevoluteJoint*>(oldJoint);

            Transform oldLink1_X_oldLink2 = oldJointRevolute->getRestTransform(oldLink1,oldLink2);
            Transform newLink1_X_newLink2 = newLink1_X_oldLink1*oldLink1_X_oldLink2*newLink2_X_oldLink2.inverse();

            Axis rotationAxis_wrt_newLink2 = newLink2_X_oldLink2*oldJointRevolute->getAxis(oldLink2);

            RevoluteJoint* newJointRevolute = new RevoluteJoint(*oldJointRevolute);

            newJointRevolute->setAttachedLinks(newLink1,newLink2);
            newJointRevolute->setRestTransform(newLink1_X_newLink2);
            newJointRevolute->setAxis(rotationAxis_wrt_newLink2, newLink2);

            newJoint = (IJointPtr) newJointRevolute;
        }
        else if( dynamic_cast<const PrismaticJoint*>(oldJoint) )
        {
            const PrismaticJoint* oldJointPrismatic = dynamic_cast<const PrismaticJoint*>(oldJoint);
            
            Transform oldLink1_X_oldLink2 = oldJointPrismatic->getRestTransform(oldLink1,oldLink2);
            Transform newLink1_X_newLink2 = newLink1_X_oldLink1*oldLink1_X_oldLink2*newLink2_X_oldLink2.inverse();
            
            Axis prismaticAxis_wrt_newLink2 = newLink2_X_oldLink2*oldJointPrismatic->getAxis(oldLink2);
            
            PrismaticJoint* newJointPrismatic = new PrismaticJoint(*oldJointPrismatic);
            
            newJointPrismatic->setAttachedLinks(newLink1,newLink2);
            newJointPrismatic->setRestTransform(newLink1_X_newLink2);
            newJointPrismatic->setAxis(prismaticAxis_wrt_newLink2, newLink2);
            
            newJoint = (IJointPtr) newJointPrismatic;
        }
        else
        {
            std::cerr << "[ERROR] createReducedModel error : "
                      << " processing joint that is not revolute, prismatic or fixed. "
                      << std::endl;
            return false;
        }

        reducedModel.addJoint(jointName,newJoint);

        // The pointer is cloned in the addJoint call,
        // we need to delete the one that we allocated ourself
        delete newJoint;
    }

    return ok;
}

bool createModelWithNormalizedJointNumbering(const Model& model,
                                             const std::string& baseForNormalizedJointNumbering,
                                             Model& normalizedModel)
{
    if (!model.isLinkNameUsed(baseForNormalizedJointNumbering))
    {
        std::cerr << "[ERROR] createModelWithNormalizedJointNumbering error : "
                      << " Link " << baseForNormalizedJointNumbering << " not found in the input model"
                      << std::endl;
        return false;
    }

    Traversal traversal;
    model.computeFullTreeTraversal(traversal, model.getLinkIndex(baseForNormalizedJointNumbering));

    // Ordering for non-fixed joints
    std::vector<std::string> jointOrderingNonFixed;

    // Ordering for fixed joints
    std::vector<std::string> jointOrderingFixed;

    // Iterate with the traversal to compute the normalized order
    for(TraversalIndex traversalEl=1; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        IJointConstPtr visitedJoint = traversal.getParentJoint(traversalEl);
        if (visitedJoint->getNrOfDOFs() !=0)
        {
            jointOrderingNonFixed.push_back(model.getJointName(visitedJoint->getIndex()));
        }
        else
        {
            jointOrderingFixed.push_back(model.getJointName(visitedJoint->getIndex()));
        }
    }

    // Compute complete ordering
    std::vector<std::string> jointOrdering;
    jointOrdering.insert(jointOrdering.end(), jointOrderingNonFixed.begin(), jointOrderingNonFixed.end());
    jointOrdering.insert(jointOrdering.end(), jointOrderingFixed.begin(), jointOrderingFixed.end());

    assert(jointOrdering.size() == model.getNrOfJoints());

    // Create the new model
    bool ok = createReducedModel(model, jointOrdering, normalizedModel);

    return ok;
}


}
