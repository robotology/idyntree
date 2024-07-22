// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/FreeFloatingState.h>

#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/PrismaticJoint.h>

#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

#include <cassert>
#include <unordered_map>
#include <set>
#include <vector>


namespace iDynTree
{

/**
 * The condition for a link to be classified as "fake link" are:
 *  * The link has a zero mass.
 *  * The link is a leaf, i.e. it is connected to only one neighbor.
 *  * The link is connected to its only neighbor with a fixed joint.
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

void computeTransformToTraversalBaseWithAdditionalTransform(const Model& fullModel,
                                                            const Traversal& subModelTraversal,
                                                            const JointPosDoubleArray& jointPos,
                                                                  LinkPositions& traversalBase_H_link,
                                                            const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink)
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
            // an identity transform, or a given transform if the link is in newLink_H_oldLink
            auto it = newLink_H_oldLink.find(fullModel.getLinkName(visitedLink->getIndex()));

            if (it != newLink_H_oldLink.end())
            {
                traversalBase_H_link(visitedLink->getIndex()) = it->second;
            }
            else
            {
                traversalBase_H_link(visitedLink->getIndex()) = iDynTree::Transform::Identity();
            }
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

void computeTransformToSubModelBaseWithAdditionalTransform(const Model& fullModel,
                                    const SubModelDecomposition& subModelDecomposition,
                                    const JointPosDoubleArray& jointPos,
                                          LinkPositions& subModelBase_H_link,
                                    const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink)
{
    for(size_t subModel = 0;
               subModel < subModelDecomposition.getNrOfSubModels();
               subModel++ )
    {
        computeTransformToTraversalBaseWithAdditionalTransform(fullModel,
                                        subModelDecomposition.getTraversal(subModel),
                                        jointPos,
                                        subModelBase_H_link,
                                        newLink_H_oldLink);
    }
}

void reducedModelAddAdditionalFrames(const Model& fullModel,
                                           Model& reducedModel,
                                     const std::string linkInReducedModel,
                                     const Traversal& linkSubModel,
                                     const FreeFloatingPos& pos,
                                           LinkPositions& subModelBase_X_link,
                                     const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink
                                     )
{
    // First compute the transform between each link in the submodel and the submodel base
    computeTransformToTraversalBaseWithAdditionalTransform(fullModel,linkSubModel,pos.jointPos(),subModelBase_X_link,newLink_H_oldLink);

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
                                const LinkPositions& subModelBase_X_link)
{
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

// This function is a private function that is not exposed in the headers, but it used as a backend
// of both:
//  * createReducedModel : function to create a reduced model given the specified joints
//  * moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink: function to make sure a model is URDF compatible
// The logic is similar to the createReducedModel, but as an additional option this function takes in input
// a std::vector<iDynTree::Transform> newLink_H_oldLink vector (of size fullModel.getNrOfLinks() that can be used
// to specify an optional additional transform of the final link used in the "reduced model"
bool createReducedModelAndChangeLinkFrames(const Model& fullModel,
                                           const std::vector< std::string >& jointsInReducedModel,
                                           Model& reducedModel,
                                           const std::unordered_map<std::string, double>& removedJointPositions,
                                           const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink,
                                           bool addOriginalLinkFrameWith_original_frame_suffix)
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

    // The position for the joint removed from the model is supposed to be 0
    FreeFloatingPos jointPos(fullModel);

    // \todo used an appropriate method here
    for(JointIndex jntIdx=0; jntIdx < fullModel.getNrOfJoints(); jntIdx++)
    {
        // Get nr of DOFs for joint
        size_t nrOfDofs = fullModel.getJoint(jntIdx)->getNrOfDOFs();

        // Nothing to do if the joint is fixed
        if (nrOfDofs == 0)
        {
            continue;
        }

        // If the joint has 1 DOF, either use the value specified in removedJointPositions
        // or if no value is found in removedJointPositions, use 0
        if (nrOfDofs == 1)
        {
            auto it = removedJointPositions.find(fullModel.getJointName(jntIdx));

            if (it != removedJointPositions.end())
            {
                jointPos.jointPos()(fullModel.getJoint(jntIdx)->getPosCoordsOffset()) = it->second;
            }
            else
            {
                jointPos.jointPos()(fullModel.getJoint(jntIdx)->getPosCoordsOffset()) = 0.0;
            }

            continue;
        }

        // If the joint has nrOfDofs > 1, we do not support specifying it in removedJointPositions
        if (nrOfDofs > 1)
        {
            for (size_t j=0; j < nrOfDofs; j++)
            {
                jointPos.jointPos()(fullModel.getJoint(jntIdx)->getPosCoordsOffset() + j) = 0.0;
            }
        }
    }

    LinkPositions subModelBase_X_link(fullModel);
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

        // The link inertia is just its own CRBA in the submodel, eventually considering the offset
        // between the link frame and the new link frame
        iDynTree::Transform newLink_H_oldLink_trans;
        auto it = newLink_H_oldLink.find(fullModel.getLinkName(linkFullModelIndex));

        if (it != newLink_H_oldLink.end())
        {
            newLink_H_oldLink_trans = it->second;
        }
        else
        {
            newLink_H_oldLink_trans = iDynTree::Transform::Identity();
        }

        iDynTree::SpatialInertia linkInertia = newLink_H_oldLink_trans*crbas(linkFullModelIndex);

        Link newLinkForReducedModel;
        newLinkForReducedModel.setInertia(linkInertia);

        // Add the new link to the reducedModel
        reducedModel.addLink(linkName,newLinkForReducedModel);

        // Get not-base links of the submodel and transform
        // them in additional frames in the reduced model
        // and get additional frames and copy them to reduced e
        // This function is the one that computes the subModelBase_X_link
        // that contains the transform between the link in the original model
        // and the corresponding link in the transformed model, that is used in the rest of the function
        // As this quantity is influenced by newLink_H_oldLink, this is passed along
        reducedModelAddAdditionalFrames(fullModel,reducedModel,
                                        linkName,subModels.getTraversal(linkInReducedModel),
                                        jointPos,subModelBase_X_link,newLink_H_oldLink);

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

    // We then handle the sensors
    // make sure that reducedModel.sensors() is empty
    assert(reducedModel.sensors().getNrOfSensors(SIX_AXIS_FORCE_TORQUE) == 0);
    assert(reducedModel.sensors().getNrOfSensors(ACCELEROMETER) == 0);
    assert(reducedModel.sensors().getNrOfSensors(GYROSCOPE) == 0);

    // Process first F/T sensors
    for (auto it = fullModel.sensors().sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE); it.isValid(); ++it) {
        Sensor* s = *it;
        JointSensor* jointSens = dynamic_cast<JointSensor*>(s);

        // If the sensor is a joint sensor
        if (jointSens) {
            // The parent joint can be present in the reduced model, or it could have been assigned to
            // a submodel after the reduction
            std::string parentJointName = jointSens->getParentJoint();

            // If the parent's joint is present in the model
            if (reducedModel.isJointNameUsed(parentJointName)) {
                // If we add the sensor to the new sensors list, we have to upgrade the indices
                SixAxisForceTorqueSensor* sensorCopy;
                sensorCopy = static_cast<SixAxisForceTorqueSensor*>(jointSens->clone());

                std::string oldFirstLinkName = sensorCopy->getFirstLinkName();
                std::string oldSecondLinkName = sensorCopy->getSecondLinkName();

                iDynTree::LinkIndex firstLinkIndex = reducedModel.getLinkIndex(oldFirstLinkName);
                iDynTree::LinkIndex secondLinkIndex = reducedModel.getLinkIndex(oldSecondLinkName);

                // The reduced model contains both the links attached to the joint
                // No particular operations are required in this case
                if ((firstLinkIndex != iDynTree::LINK_INVALID_INDEX) &&
                    (secondLinkIndex != iDynTree::LINK_INVALID_INDEX)) {
                    // Update indices
                    sensorCopy->updateIndices(reducedModel);
                    // Add the sensor to the reduced model
                    reducedModel.sensors().addSensor(*sensorCopy);
                }
                // If one of the links attached to the joint has been lumped in the reduced model,
                // updating the transform is required
                else if (firstLinkIndex != iDynTree::LINK_INVALID_INDEX) {
                    // The secondLink has been lumped
                    // Get the link to which it was merged
                    FrameIndex frameIndexOfSecondLink = reducedModel.getFrameIndex(oldSecondLinkName);
                    LinkIndex newSecondLinkIndex = reducedModel.getFrameLink(frameIndexOfSecondLink);

                    // Update the transform. It requires two steps:
                    // New second link (reducedModel) -> Old second link (fullModel) -> jointSens frame
                    Transform oldSecondLinkInFullModel_H_sensorFrame;

                    sensorCopy->getLinkSensorTransform(sensorCopy->getSecondLinkIndex(),
                                                       oldSecondLinkInFullModel_H_sensorFrame);

                    Transform newSecondLinkInReducedModel_H_oldSecondLinkInFullModel =
                        reducedModel.getFrameTransform(frameIndexOfSecondLink);

                    // Get the name of the new second link (to which the old one has been lumped)
                    std::string newSecondLinkName = reducedModel.getLinkName(newSecondLinkIndex);

                    // Set the name of the new secondLink and update its index
                    sensorCopy->setSecondLinkName(newSecondLinkName);
                    sensorCopy->updateIndices(reducedModel);

                    // Update the transform
                    sensorCopy->setSecondLinkSensorTransform(sensorCopy->getSecondLinkIndex(),
                                                            newSecondLinkInReducedModel_H_oldSecondLinkInFullModel*oldSecondLinkInFullModel_H_sensorFrame);

                    // Update the appliedWrenchLink
                    if (fullModel.getLinkName(sensorCopy->getAppliedWrenchLink()) == oldSecondLinkName) {
                        sensorCopy->setAppliedWrenchLink(reducedModel.getLinkIndex(newSecondLinkName));
                    }

                    // Add the sensor to the reduced model
                    reducedModel.sensors().addSensor(*sensorCopy);
                }
                else if (secondLinkIndex != iDynTree::LINK_INVALID_INDEX) {
                    // The firstLink has been lumped
                    // Get the link to which it was merged
                    FrameIndex frameIndexOfFirstLink = reducedModel.getFrameIndex(oldFirstLinkName);
                    LinkIndex newFirstLinkIndex = reducedModel.getFrameLink(frameIndexOfFirstLink);

                    // Update the transform. It requires two steps:
                    // New first link (reducedModel) -> Old first link (fullModel) -> jointSens frame
                    Transform oldFirstLinkInFullModel_H_sensorFrame;

                    sensorCopy->getLinkSensorTransform(sensorCopy->getFirstLinkIndex(),
                                                       oldFirstLinkInFullModel_H_sensorFrame);

                    Transform newFirstLinkInReducedModel_H_oldFirstLinkInFullModel =
                        reducedModel.getFrameTransform(frameIndexOfFirstLink);

                    // Get the name of the new first link (to which the old one has been lumped)
                    std::string newFirstLinkName = reducedModel.getLinkName(newFirstLinkIndex);

                    // Set the name of the new firstLink and update its index
                    sensorCopy->setFirstLinkName(newFirstLinkName);
                    sensorCopy->updateIndices(reducedModel);

                    // Update the transform
                    sensorCopy->setFirstLinkSensorTransform(sensorCopy->getFirstLinkIndex(),
                                                            newFirstLinkInReducedModel_H_oldFirstLinkInFullModel*oldFirstLinkInFullModel_H_sensorFrame);

                    // Update the appliedWrenchLink
                    if (fullModel.getLinkName(sensorCopy->getAppliedWrenchLink()) == oldFirstLinkName) {
                        sensorCopy->setAppliedWrenchLink(reducedModel.getLinkIndex(newFirstLinkName));
                    }

                    // Add the sensor to the reduced model
                    reducedModel.sensors().addSensor(*sensorCopy);
                }
                else {
                    std::stringstream ss;
                    ss << "The links related to the joint sensor attached on " << parentJointName << " have an invalid index" << std::endl;
                    reportError("", "createReducedModelAndSensors", ss.str().c_str());
                    delete sensorCopy;
                    return false;
                }

                delete sensorCopy;
            }
        }
        else {
            std::stringstream ss;
            ss << "The processed FT sensor couldn't be cast as a joint sensor" << std::endl;
            reportWarning("", "createReducedModelAndSensors", ss.str().c_str());

        }
    }

    // Then all link sensors
    for (SensorsList::const_iterator it = fullModel.sensors().allSensorsIterator(); it.isValid(); ++it)
    {
        Sensor *s = *it;

        // This should select only link sensors
        LinkSensor *linkSens = dynamic_cast<LinkSensor*>(s);
        if( linkSens )
        {
            std::string sensorLinkInFullModel = linkSens->getParentLink();

            // If the link to wicht the sensors is attached (parentLink) is also in the reduced model, we can just copy the sensor to the reduced sensors models
            if( reducedModel.isLinkNameUsed(sensorLinkInFullModel) )
            {
                // update the link index of the reduced model
                LinkSensor *sensorInReducedModel = static_cast<LinkSensor*>(linkSens->clone());
                if (!sensorInReducedModel || !sensorInReducedModel->updateIndices(reducedModel)) {
                    reportError("","createReducedModelAndSensors", "Failed to duplicate LinkSensor and update indices");
                    return false;
                }
                reducedModel.sensors().addSensor(*sensorInReducedModel);
                delete sensorInReducedModel;
            }
            else
            {
                // Otherwise there should be a additional frame in the reduced model named like the sensor link in the full model
                if( !reducedModel.isFrameNameUsed(sensorLinkInFullModel) )
                {
                    std::stringstream ss;
                    ss << "additional frame " << sensorLinkInFullModel << " is not in the reduced model, reducing sensors failed" << std::endl;
                    reportError("","createReducedModelAndSensors",ss.str().c_str());
                    return false;
                }


                FrameIndex sensorLinkAdditionalFrameIndexInReducedModel = reducedModel.getFrameIndex(sensorLinkInFullModel);
                LinkIndex sensorLinkInReducedModelIdx = reducedModel.getFrameLink(sensorLinkAdditionalFrameIndexInReducedModel);

                std::string sensorLinkInReducedModel = reducedModel.getLinkName(sensorLinkInReducedModelIdx);

                // If we found the original link sensor as an additonal frame, we can compute the pose of the sensor w.r.t. to the new link to which it is attached
                Transform sensorLinkInReducedModel_H_sensorLinkInFullModel = reducedModel.getFrameTransform(sensorLinkAdditionalFrameIndexInReducedModel);
                Transform sensorLinkInFullModel_H_sensorFrame              = linkSens->getLinkSensorTransform();

                Transform sensorLinkInReducedModel_H_sensorFrame = sensorLinkInReducedModel_H_sensorLinkInFullModel*sensorLinkInFullModel_H_sensorFrame;

                // Copy the sensor to modify it
                LinkSensor* sensorCopy = (LinkSensor*)linkSens->clone();

                // Update the pose
                sensorCopy->setLinkSensorTransform(sensorLinkInReducedModel_H_sensorFrame);

                // Update the link name and indices
                sensorCopy->setParentLink(sensorLinkInReducedModel);
                sensorCopy->setParentLinkIndex(reducedModel.getLinkIndex(sensorLinkInReducedModel));

                reducedModel.sensors().addSensor(*sensorCopy);

                delete sensorCopy;
            }

        }
    }

    return ok;
}

bool createReducedModel(const Model& fullModel,
                        const std::vector< std::string >& jointsInReducedModel,
                        Model& reducedModel,
                        const std::unordered_map<std::string, double>& removedJointPositions)
{
    // We do not want to move the link frames in createReducedModel
    std::unordered_map<std::string, iDynTree::Transform> newLink_H_oldLink;
    bool addOriginalLinkFrameWith_original_frame_suffix = false;
    return createReducedModelAndChangeLinkFrames(fullModel, jointsInReducedModel, reducedModel, removedJointPositions, newLink_H_oldLink, addOriginalLinkFrameWith_original_frame_suffix);
}

bool createReducedModel(const Model& fullModel,
                        const std::vector< std::string >& jointsInReducedModel,
                        Model& reducedModel)
{
    std::unordered_map<std::string, double> emptyRemovedJointPositions;
    return createReducedModel(fullModel, jointsInReducedModel, reducedModel, emptyRemovedJointPositions);
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


bool extractSubModel(const iDynTree::Model& fullModel, const iDynTree::Traversal& subModelTraversal,
                     iDynTree::Model& outputSubModel)
{
    size_t nrOfLinksInReducedModel = subModelTraversal.getNrOfVisitedLinks();

    LinkPositions subModelBase_X_link(fullModel);

    for (size_t subModelTraversalEl = 0; subModelTraversalEl < nrOfLinksInReducedModel; subModelTraversalEl++)
    {
        // Add the new link to the reducedModel

        LinkConstPtr visitedLink = subModelTraversal.getLink(subModelTraversalEl);

        outputSubModel.addLink(fullModel.getLinkName(visitedLink->getIndex()), *visitedLink);

        bool isLinkBaseOfTreeTraversal = (subModelTraversalEl == 0);

        if (!isLinkBaseOfTreeTraversal)
        {
            // Add the new joint to the reducedModel

            IJointConstPtr visitedParentJoint = subModelTraversal.getParentJoint(subModelTraversalEl);

            std::string jointName = fullModel.getJointName(visitedParentJoint->getIndex());

            // We get the two links that the joint connects in the old
            // full model
            JointIndex fullModelJointIndex = fullModel.getJointIndex(jointName);

            IJointConstPtr oldJoint = fullModel.getJoint(fullModelJointIndex);
            const std::string link1 = fullModel.getLinkName(subModelTraversal.getParentLinkIndexFromJointIndex(fullModel, fullModelJointIndex));
            const std::string link2 = fullModel.getLinkName(visitedLink->getIndex());

            outputSubModel.addJoint(link1, link2, jointName, oldJoint);
        }

        auto visitedLinkIndex = visitedLink->getIndex();

        // Add the additional frames
        std::vector<FrameIndex> frameIndeces;
        fullModel.getLinkAdditionalFrames(visitedLinkIndex, frameIndeces);

        // For all the link of the submodel, transfer their additional frame
        // to the link in the reduced model

        for(size_t i = 0; i < frameIndeces.size(); i++)
        {
            FrameIndex additionalFrame = frameIndeces[i];
            std::string additionalFrameName = fullModel.getFrameName(additionalFrame);

            Transform visitedLink_H_additionalFrame  = fullModel.getFrameTransform(additionalFrame);
            Transform subModelBase_H_visitedLink     = subModelBase_X_link(visitedLinkIndex);
            Transform subModelBase_H_additionalFrame =
                subModelBase_H_visitedLink*visitedLink_H_additionalFrame;

            outputSubModel.addAdditionalFrameToLink(fullModel.getFrameName(visitedLinkIndex),
                                                    additionalFrameName, subModelBase_H_additionalFrame);
        }
    }

    return  true;
}

void addValidNamesToAllSolidShapesHelper(iDynTree::Model& outputModel,
                                         std::vector<std::vector<SolidShape *>> solidShapes,
                                         std::string suffix)
{
    for (iDynTree::LinkIndex lnkIndex = 0; lnkIndex < outputModel.getNrOfLinks(); lnkIndex++)
    {
        // Check the number of solid shapes for this link
        size_t nrOfSolidShapes = solidShapes[lnkIndex].size();

        // If there are not shapes, we just continue
        if (nrOfSolidShapes==0)
        {
            continue;
        }

        if (nrOfSolidShapes == 1)
        {
            // In this case, if there is no valid name the shape will just be called <linkName>_collision or <linkName>_visual
            if (!solidShapes[lnkIndex][0]->isNameValid())
            {
                std::string shapeName = outputModel.getLinkName(lnkIndex) + "_" + suffix;
                solidShapes[lnkIndex][0]->setName(shapeName);
            }
        }


        if (nrOfSolidShapes > 1)
        {
            bool isThereAtLeastAValidName = false;
            // In this case, if there is no valid name the shape will just be called <linkName>_collision_0, <linkName>_collision_1, etc etc
            for(int shapeIdx=0; shapeIdx < solidShapes[lnkIndex].size(); shapeIdx++)
            {
                if (solidShapes[lnkIndex][shapeIdx]->isNameValid())
                {
                    isThereAtLeastAValidName = true;
                }
            }

            // In case all names are invalid, set names
            if (!isThereAtLeastAValidName)
            {
                for(int shapeIdx=0; shapeIdx < solidShapes[lnkIndex].size(); shapeIdx++)
                {
                    std::string shapeName = outputModel.getLinkName(lnkIndex) + "_" + suffix + "_" + std::to_string(shapeIdx);
                    solidShapes[lnkIndex][shapeIdx]->setName(shapeName);
                }
            }
        }
    }

}

bool addValidNamesToAllSolidShapes(const iDynTree::Model& inputModel,
                                   iDynTree::Model& outputModel)
{
    outputModel = inputModel;

    auto& visualSolidShapes = outputModel.visualSolidShapes().getLinkSolidShapes();
    auto& collisionSolidShapes = outputModel.collisionSolidShapes().getLinkSolidShapes();

    addValidNamesToAllSolidShapesHelper(outputModel, visualSolidShapes, "visual");
    addValidNamesToAllSolidShapesHelper(outputModel, collisionSolidShapes, "collision");

    return true;
}

bool moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink(const iDynTree::Model& inputModel,
                                                                 iDynTree::Model& outputModel)
{

    // In this transformer we do not remove any joint, so the list of considered joints is exactly the list of joints in the input model
    std::vector<std::string> consideredJoints;
    for(iDynTree::JointIndex jntIdx=0; jntIdx < inputModel.getNrOfJoints(); jntIdx++)
    {
        consideredJoints.push_back(inputModel.getJointName(jntIdx));
    }

    // As we do not remove any joint, this map is empty
    std::unordered_map<std::string, double> removedJointPositions;

    // We now need to compute the newLink_H_oldLink transform for each link we need to transform
    std::unordered_map<std::string, iDynTree::Transform> newLink_H_oldLink;

    // First of all, compute the traversal for the default base of the model
    iDynTree::Traversal fullTraversal;
    bool ok = inputModel.computeFullTreeTraversal(fullTraversal);

    if (!ok)
    {
        return false;
    }

    // We start from 1 as the first link does not have any joint to the aparent link
    for (iDynTree::TraversalIndex trvIdx=1; trvIdx < static_cast<TraversalIndex>(fullTraversal.getNrOfVisitedLinks()); trvIdx++)
    {
        LinkConstPtr childLink = fullTraversal.getLink(trvIdx);
        std::string childLinkName = inputModel.getLinkName(childLink->getIndex());
        LinkConstPtr parentLink = fullTraversal.getParentLink(trvIdx);
        IJointConstPtr jointToParent = fullTraversal.getParentJoint(trvIdx);


        if (jointToParent->getNrOfDOFs() != 0)
        {
            iDynTree::Axis axis;

            // Check that the axis of the joint is supported by URDF
            if (dynamic_cast<const RevoluteJoint*>(jointToParent))
            {
                const RevoluteJoint* revJoint = dynamic_cast<const RevoluteJoint*>(jointToParent);
                axis = revJoint->getAxis(childLink->getIndex());
            }
            else if (dynamic_cast<const PrismaticJoint*>(jointToParent))
            {
                const PrismaticJoint* prismJoint = dynamic_cast<const PrismaticJoint*>(jointToParent);
                axis = prismJoint->getAxis(childLink->getIndex());
            }

            // If the axis is not URDF-compatible, move it to ensure that the new link frame lays on the
            // The threshold of 1e-7 is the same used in URDFStringFromModel, if you change it here also change it there
            double distanceBetweenAxisAndOrigin = axis.getDistanceBetweenAxisAndPoint(iDynTree::Position::Zero());
            if (distanceBetweenAxisAndOrigin > 1e-7)
            {

                // First of all, we get the point on the child axis closest to the existing origin
                iDynTree::Position oldLink_o_newLink = axis.getPointOnAxisClosestToGivenPoint(iDynTree::Position::Zero());

                // We invert the sign
                iDynTree::Position newLink_o_oldLink = -oldLink_o_newLink;


                newLink_H_oldLink[childLinkName] = iDynTree::Transform(iDynTree::Rotation::Identity(), newLink_o_oldLink);
            }
        }
    }

    bool addOriginalLinkFrameWith_original_frame_suffix = true;
    bool okReduced = createReducedModelAndChangeLinkFrames(inputModel, consideredJoints, outputModel,
                                                 removedJointPositions, newLink_H_oldLink, addOriginalLinkFrameWith_original_frame_suffix);

    if (okReduced)
    {
        // In general createReducedModelAndChangeLinkFrames do not preserve the existance of the base link,
        // but as in this case the link existence is preserved, we also preserve the default base link
        outputModel.setDefaultBaseLink(outputModel.getLinkIndex(inputModel.getLinkName(inputModel.getDefaultBaseLink())));
        return true;
    }
    else
    {
        return false;
    }
}

}
