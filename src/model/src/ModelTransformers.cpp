// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Model.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/FreeFloatingState.h>

#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/RevoluteSO2Joint.h>
#include <iDynTree/SphericalJoint.h>

#include <iDynTree/Sensors.h>
#include <iDynTree/SixAxisForceTorqueSensor.h>

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <set>
#include <vector>
#include <iDynTree/Axis.h>
#include <iDynTree/Direction.h>


namespace iDynTree
{

struct SphericalJointConversion {
    std::string originalJointName;
    std::string parentLinkName;
    std::string childLinkName;
    Transform originalJointTransform;

    // Generated components
    std::string fakeLinkName1;
    std::string fakeLinkName2;
    std::string revJointName1; // X-axis rotation
    std::string revJointName2; // Y-axis rotation
    std::string revJointName3; // Z-axis rotation
};

struct ThreeRevoluteJointPattern {
    std::string parentLinkName;
    std::string intermediateLinkName1;
    std::string intermediateLinkName2;
    std::string childLinkName;

    std::string revJointName1; // X-axis rotation
    std::string revJointName2; // Y-axis rotation
    std::string revJointName3; // Z-axis rotation

    Transform parentLink_H_childLink;
    Position jointCenter; // In parent link frame
};

// Helper function to check if three vectors are orthogonal
bool areVectorsOrthogonal(const Direction& v1, const Direction& v2, const Direction& v3, double tolerance) {
    return v1.isPerpendicular(v2, tolerance) &&
           v1.isPerpendicular(v3, tolerance) &&
           v2.isPerpendicular(v3, tolerance);
}

// Helper function to find intersection point of three lines
bool findAxisIntersectionPoint(const Axis& axis1, const Axis& axis2, const Axis& axis3,
                              Position& intersectionPoint, double tolerance) {
    // For simplicity, we'll check if all axes pass through the same point
    // by checking if the distance from each axis to a common point is within tolerance

    // Use axis1's origin as reference point
    Position refPoint = axis1.getOrigin();

    double dist2 = axis2.getDistanceBetweenAxisAndPoint(refPoint);
    double dist3 = axis3.getDistanceBetweenAxisAndPoint(refPoint);

    if (dist2 < tolerance && dist3 < tolerance) {
        intersectionPoint = refPoint;
        return true;
    }

    return false;
}

IJointConstPtr getJointFromLinks(const Model& model, LinkConstPtr linkA, LinkConstPtr linkB)
{
    for(JointIndex jntIndex = 0; jntIndex < model.getNrOfJoints(); jntIndex++)
    {
        auto joint = model.getJoint(jntIndex);
        if ( (joint->getFirstAttachedLink() == linkA->getIndex() && joint->getSecondAttachedLink() == linkB->getIndex()) || (joint->getFirstAttachedLink() == linkB->getIndex() && joint->getSecondAttachedLink() == linkA->getIndex()) )
        {
            return joint;
        }
    }
    return nullptr;
}

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

void addAdditionalFrameIfAllowed(Model& reducedModel,
                                 const std::string linkInReducedModel,
                                 const std::string additionalFrameName,
                                 const Transform& subModelBase_H_additionalFrame,
                                 bool includeAllAdditionalFrames,
                                 const std::vector<std::string>& allowedAdditionalFrames)
{
    bool shouldWeAddTheAdditionalFrame = true;

    // Check if we need to add the additional frame or not
    if (!includeAllAdditionalFrames)
    {
        // If allowedAdditionalFrames has a value, we only need to add additional frames specified ther
        if (std::find(allowedAdditionalFrames.begin(), allowedAdditionalFrames.end(), additionalFrameName) == allowedAdditionalFrames.end())
        {
            shouldWeAddTheAdditionalFrame = false;
        }
    }

    if (shouldWeAddTheAdditionalFrame)
    {
        reducedModel.addAdditionalFrameToLink(linkInReducedModel,additionalFrameName,
                                               subModelBase_H_additionalFrame);
    }
}

void reducedModelAddAdditionalFrames(const Model& fullModel,
                                           Model& reducedModel,
                                     const std::string linkInReducedModel,
                                     const Traversal& linkSubModel,
                                     const FreeFloatingPos& pos,
                                           LinkPositions& subModelBase_X_link,
                                     const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink,
                                     bool includeAllAdditionalFrames,
                                     const std::vector<std::string>& allowedAdditionalFrames)
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

            addAdditionalFrameIfAllowed(reducedModel, linkInReducedModel,
                                        additionalFrameName, subModelBase_H_additionalFrame,
                                        includeAllAdditionalFrames, allowedAdditionalFrames);
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

            addAdditionalFrameIfAllowed(reducedModel, linkInReducedModel,
                                        additionalFrameName, subModelBase_H_additionalFrame,
                                        includeAllAdditionalFrames, allowedAdditionalFrames);
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
//  * removeAdditionalFramesFromModel: function to remove additional frames from a URDF
//
// The logic is similar to the createReducedModel, but with additional options:
// * std::vector<iDynTree::Transform> newLink_H_oldLink vector (of size fullModel.getNrOfLinks() that can be used
//    to specify an optional additional transform of the final link used in the "reduced model"
// * includeAllAdditionalFrames, std::vector<std::string> : If includeAllAdditionalFrames is true,
//   all the additional frames are of the input model are copied in the reduced model, if includeAllAdditionalFrames is true
//   is True only the additional frames with the name contained in allowedAdditionalFrames are copied to the reduce model
bool createReducedModelAndChangeLinkFrames(const Model& fullModel,
                                           const std::vector< std::string >& jointsInReducedModel,
                                           Model& reducedModel,
                                           const std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
                                           const std::unordered_map<std::string, iDynTree::Transform>& newLink_H_oldLink,
                                           bool addOriginalLinkFrameWith_original_frame_suffix,
                                           bool includeAllAdditionalFrames,
                                           const std::vector<std::string>& allowedAdditionalFrames)
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

    for(JointIndex jntIdx=0; jntIdx < fullModel.getNrOfJoints(); jntIdx++)
    {
        // Get nr of position coordinates for joint (e.g., 4 for spherical joint quaternion)
        size_t nrOfPosCoords = fullModel.getJoint(jntIdx)->getNrOfPosCoords();
        size_t nrOfDofs = fullModel.getJoint(jntIdx)->getNrOfDOFs();

        // Nothing to do if the joint is fixed
        if (nrOfPosCoords == 0)
        {
            continue;
        }

        std::string jointName = fullModel.getJointName(jntIdx);
        auto it = removedJointPositions.find(jointName);

        if (it != removedJointPositions.end())
        {
            // Check if the provided vector has the correct size
            if (it->second.size() != nrOfPosCoords)
            {
                std::cerr << "[ERROR] createReducedModel error : "
                            << " joint " << jointName << " expects " << nrOfPosCoords
                            << " position coordinates, but " << it->second.size()
                            << " were provided in removedJointPositions."
                            << std::endl;
                return false;
            }

            // Set the joint position coordinates from the provided vector
            size_t posOffset = fullModel.getJoint(jntIdx)->getPosCoordsOffset();
            for (size_t i = 0; i < nrOfPosCoords; i++)
            {
                jointPos.jointPos()(posOffset + i) = it->second[i];
            }
        }
        else
        {
            // If no values provided, set to rest position
            fullModel.getJoint(jntIdx)->setJointPosCoordsToRest(jointPos.jointPos());
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
                                        jointPos,subModelBase_X_link,newLink_H_oldLink,
                                        includeAllAdditionalFrames,allowedAdditionalFrames);

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
        else if (dynamic_cast<const RevoluteSO2Joint*>(oldJoint))
        {
            const RevoluteSO2Joint* oldJointRevoluteSO2 = dynamic_cast<const RevoluteSO2Joint*>(oldJoint);

            Transform oldLink1_X_oldLink2 = oldJointRevoluteSO2->getRestTransform(oldLink1, oldLink2);
            Transform newLink1_X_newLink2 = newLink1_X_oldLink1 * oldLink1_X_oldLink2 * newLink2_X_oldLink2.inverse();

            Axis rotationAxis_wrt_newLink2 = newLink2_X_oldLink2 * oldJointRevoluteSO2->getAxis(oldLink2);

            RevoluteSO2Joint* newJointRevoluteSO2 = new RevoluteSO2Joint(*oldJointRevoluteSO2);

            newJointRevoluteSO2->setAttachedLinks(newLink1, newLink2);
            newJointRevoluteSO2->setRestTransform(newLink1_X_newLink2);
            newJointRevoluteSO2->setAxis(rotationAxis_wrt_newLink2, newLink2);

            newJoint = (IJointPtr)newJointRevoluteSO2;
        }
        else if (dynamic_cast<const SphericalJoint*>(oldJoint))
        {
            const SphericalJoint* oldJointSpherical = dynamic_cast<const SphericalJoint*>(oldJoint);

            Transform oldLink1_X_oldLink2 = oldJointSpherical->getRestTransform(oldLink1, oldLink2);
            Transform newLink1_X_newLink2 = newLink1_X_oldLink1 * oldLink1_X_oldLink2 * newLink2_X_oldLink2.inverse();

            SphericalJoint* newJointSpherical = new SphericalJoint(*oldJointSpherical);

            newJointSpherical->setAttachedLinks(newLink1, newLink2);
            newJointSpherical->setRestTransform(newLink1_X_newLink2);

            // Update joint center to account for coordinate frame changes
            // Transform center in first link (newLink1) coordinate system
            Position oldCenterInFirstLink = oldJointSpherical->getJointCenter(oldLink1);
            Position newCenterInFirstLink;
            newCenterInFirstLink = newLink1_X_oldLink1 * oldCenterInFirstLink;
            newJointSpherical->setJointCenter(newLink1, newCenterInFirstLink);

            // Transform center in second link (newLink2) coordinate system
            Position oldCenterInSecondLink = oldJointSpherical->getJointCenter(oldLink2);
            Position newCenterInSecondLink;
            newCenterInSecondLink = newLink2_X_oldLink2 * oldCenterInSecondLink;
            newJointSpherical->setJointCenter(newLink2, newCenterInSecondLink);

            newJoint = (IJointPtr)newJointSpherical;
        }
        else
        {
            std::cerr << "[ERROR] createReducedModel error : "
                      << " processing joint that is not revolute, prismatic, revolute SO2, spherical or fixed. "
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
                        sensorCopy->setAppliedWrenchLink(newSecondLinkIndex);
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
                        sensorCopy->setAppliedWrenchLink(newFirstLinkIndex);
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
                        const std::vector<std::string>& jointsInReducedModel,
                        Model& reducedModel,
                        const std::unordered_map<std::string, std::vector<double>>& removedJointPositions)
{
    // We do not want to move the link frames in createReducedModel
    std::unordered_map<std::string, iDynTree::Transform> newLink_H_oldLink;
    bool addOriginalLinkFrameWith_original_frame_suffix = false;
    bool includeAllAdditionalFrames = true;
    return createReducedModelAndChangeLinkFrames(fullModel, jointsInReducedModel, reducedModel, removedJointPositions, newLink_H_oldLink, addOriginalLinkFrameWith_original_frame_suffix, includeAllAdditionalFrames, {});
}

bool createReducedModel(const Model& fullModel,
                        const std::vector< std::string >& jointsInReducedModel,
                        Model& reducedModel,
                        const std::unordered_map<std::string, double>& removedJointPositions)
{
    // Convert the double-based map to vector-based map
    std::unordered_map<std::string, std::vector<double>> removedJointPositionsVector;
    for (const auto& pair : removedJointPositions)
    {
        removedJointPositionsVector[pair.first] = {pair.second};
    }

    return createReducedModel(fullModel, jointsInReducedModel, reducedModel, removedJointPositionsVector);
}

bool createReducedModel(const Model& fullModel,
                        const std::vector< std::string >& jointsInReducedModel,
                        Model& reducedModel)
{
    std::unordered_map<std::string, std::vector<double>> emptyRemovedJointPositions;
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
    std::unordered_map<std::string, std::vector<double>> removedJointPositions;

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
    bool includeAllAdditionalFrames = true;
    bool okReduced = createReducedModelAndChangeLinkFrames(inputModel, consideredJoints, outputModel,
                                                 removedJointPositions, newLink_H_oldLink, addOriginalLinkFrameWith_original_frame_suffix, includeAllAdditionalFrames, {});

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

bool removeAdditionalFramesFromModel(const Model& modelWithAllAdditionalFrames,
                                           Model& modelWithOnlyAllowedAdditionalFrames,
                                           const std::vector<std::string> allowedAdditionalFrames)
{
    // Get list of all joints to pass as considered joints
    std::vector<std::string> consideredJoints;
    for(iDynTree::JointIndex jntIdx=0; jntIdx < modelWithAllAdditionalFrames.getNrOfJoints(); jntIdx++)
    {
        consideredJoints.push_back(modelWithAllAdditionalFrames.getJointName(jntIdx));
    }

    bool includeAllAdditionalFrames = false;
    return createReducedModelAndChangeLinkFrames(modelWithAllAdditionalFrames,
                                                 consideredJoints,
                                                 modelWithOnlyAllowedAdditionalFrames,
                                                 std::unordered_map<std::string, std::vector<double>>(),
                                                 std::unordered_map<std::string, iDynTree::Transform>(),
                                                 false,
                                                 includeAllAdditionalFrames,
                                                 allowedAdditionalFrames);

}

bool convertSphericalJointsToThreeRevoluteJoints(const Model& inputModel,
                                                 Model& outputModel,
                                                 const std::string& sphericalJointFakeLinkPrefix,
                                                 const std::string& sphericalJointRevoluteJointPrefix) {

    std::vector<SphericalJointConversion> conversions;

    // Find all spherical joints
    for (JointIndex jointIdx = 0; jointIdx < inputModel.getNrOfJoints(); jointIdx++) {
        const IJoint* joint = inputModel.getJoint(jointIdx);

        // Check if this is a spherical joint using dynamic_cast
        if (dynamic_cast<const SphericalJoint*>(joint)) {

            SphericalJointConversion conversion;
            conversion.originalJointName = inputModel.getJointName(jointIdx);

            // Get parent and child links using the correct methods
            LinkIndex parentLinkIdx = joint->getFirstAttachedLink();
            LinkIndex childLinkIdx = joint->getSecondAttachedLink();

            if (parentLinkIdx == LINK_INVALID_INDEX || childLinkIdx == LINK_INVALID_INDEX) {
                std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Invalid parent or child link index for spherical joint "
                          << conversion.originalJointName << std::endl;
                return false;
            }

            conversion.parentLinkName = inputModel.getLinkName(parentLinkIdx);
            conversion.childLinkName = inputModel.getLinkName(childLinkIdx);
            conversion.originalJointTransform = joint->getRestTransform(parentLinkIdx, childLinkIdx);

            // Generate names for fake components
            conversion.fakeLinkName1 = sphericalJointFakeLinkPrefix + conversion.originalJointName + "_link1";
            conversion.fakeLinkName2 = sphericalJointFakeLinkPrefix + conversion.originalJointName + "_link2";
            conversion.revJointName1 = sphericalJointRevoluteJointPrefix + conversion.originalJointName + "_x";
            conversion.revJointName2 = sphericalJointRevoluteJointPrefix + conversion.originalJointName + "_y";
            conversion.revJointName3 = sphericalJointRevoluteJointPrefix + conversion.originalJointName + "_z";

            // Check for name conflicts
            if (inputModel.isLinkNameUsed(conversion.fakeLinkName1) ||
                inputModel.isLinkNameUsed(conversion.fakeLinkName2)) {
                std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Generated fake link names conflict with existing links for spherical joint "
                          << conversion.originalJointName << std::endl;
                return false;
            }

            if (inputModel.isJointNameUsed(conversion.revJointName1) ||
                inputModel.isJointNameUsed(conversion.revJointName2) ||
                inputModel.isJointNameUsed(conversion.revJointName3)) {
                std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Generated revolute joint names conflict with existing joints for spherical joint "
                          << conversion.originalJointName << std::endl;
                return false;
            }

            conversions.push_back(conversion);
        }
    }

    // If no spherical joints found, return original model
    if (conversions.empty()) {
        outputModel = inputModel;
        return true;
    }

    // Create an intermediate model with the spherical joints converted
    outputModel = Model();

    // Copy all links from original model
    for (LinkIndex linkIdx = 0; linkIdx < inputModel.getNrOfLinks(); linkIdx++) {
        std::string linkName = inputModel.getLinkName(linkIdx);
        outputModel.addLink(linkName, *inputModel.getLink(linkIdx));
    }

    // Add fake links for spherical joint conversions
    for (const auto& conversion : conversions) {
        // Create zero-mass fake links
        SpatialInertia zeroInertia;
        zeroInertia.zero();

        Link fakeLinkData1;
        fakeLinkData1.setInertia(zeroInertia);

        Link fakeLinkData2;
        fakeLinkData2.setInertia(zeroInertia);

        if (outputModel.addLink(conversion.fakeLinkName1, fakeLinkData1) == LINK_INVALID_INDEX) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add fake link "
                      << conversion.fakeLinkName1 << " for spherical joint "
                      << conversion.originalJointName << std::endl;
            return false;
        }

        if (outputModel.addLink(conversion.fakeLinkName2, fakeLinkData2) == LINK_INVALID_INDEX) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add fake link "
                      << conversion.fakeLinkName2 << " for spherical joint "
                      << conversion.originalJointName << std::endl;
            return false;
        }
    }

    // Copy all non-spherical joints
    for (JointIndex jointIdx = 0; jointIdx < inputModel.getNrOfJoints(); jointIdx++) {
        const IJoint* joint = inputModel.getJoint(jointIdx);

        // Check if this is NOT a spherical joint using dynamic_cast
        if (!dynamic_cast<const SphericalJoint*>(joint)) {
            std::string jointName = inputModel.getJointName(jointIdx);
            LinkIndex parentLinkIdx = joint->getFirstAttachedLink();
            LinkIndex childLinkIdx = joint->getSecondAttachedLink();

            std::string parentLinkName = inputModel.getLinkName(parentLinkIdx);
            std::string childLinkName = inputModel.getLinkName(childLinkIdx);

            if (outputModel.addJoint(parentLinkName, childLinkName, jointName, joint) == JOINT_INVALID_INDEX) {
                std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add joint "
                          << jointName << " to intermediate model" << std::endl;
                return false;
            }
        }
    }

    // Add the three revolute joints for each spherical joint
    for (const auto& conversion : conversions) {
        Transform identity = Transform::Identity();

        // Get the spherical joint to access its center
        JointIndex originalJointIdx = inputModel.getJointIndex(conversion.originalJointName);
        const IJoint* originalJoint = inputModel.getJoint(originalJointIdx);
        const SphericalJoint* sphericalJoint = dynamic_cast<const SphericalJoint*>(originalJoint);

        if (!sphericalJoint) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to cast joint "
                      << conversion.originalJointName << " to SphericalJoint" << std::endl;
            return false;
        }

        // Get the joint center - this is where all revolute joint axes should intersect
        Position jointCenter_wrt_child = sphericalJoint->getJointCenter(sphericalJoint->getSecondAttachedLink());

        // X-axis rotation joint (parent -> fake_link1)
        Axis xAxis(Direction(1.0, 0.0, 0.0), jointCenter_wrt_child);
        RevoluteJoint xRevJoint;
        xRevJoint.setAttachedLinks(outputModel.getLinkIndex(conversion.parentLinkName), outputModel.getLinkIndex(conversion.fakeLinkName1));
        xRevJoint.setRestTransform(conversion.originalJointTransform);
        xRevJoint.setAxis(xAxis,outputModel.getLinkIndex(conversion.fakeLinkName1));

        if (outputModel.addJoint(conversion.revJointName1,
                                &xRevJoint) == JOINT_INVALID_INDEX) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add X-axis revolute joint "
                      << conversion.revJointName1 << " for spherical joint "
                      << conversion.originalJointName << std::endl;
            return false;
        }

        // Y-axis rotation joint (fake_link1 -> fake_link2)
        Axis yAxis(Direction(0.0, 1.0, 0.0), jointCenter_wrt_child);
        RevoluteJoint yRevJoint;
        yRevJoint.setAttachedLinks(outputModel.getLinkIndex(conversion.fakeLinkName1), outputModel.getLinkIndex(conversion.fakeLinkName2));
        yRevJoint.setRestTransform(identity);
        yRevJoint.setAxis(yAxis, outputModel.getLinkIndex(conversion.fakeLinkName2));

        if (outputModel.addJoint(conversion.revJointName2,
                                &yRevJoint) == JOINT_INVALID_INDEX) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add Y-axis revolute joint "
                      << conversion.revJointName2 << " for spherical joint "
                      << conversion.originalJointName << std::endl;
            return false;
        }

        // Z-axis rotation joint (fake_link2 -> child)
        Axis zAxis(Direction(0.0, 0.0, 1.0), jointCenter_wrt_child);
        RevoluteJoint zRevJoint;
        zRevJoint.setAttachedLinks(outputModel.getLinkIndex(conversion.fakeLinkName2), outputModel.getLinkIndex(conversion.childLinkName));
        zRevJoint.setRestTransform(identity);
        zRevJoint.setAxis(zAxis, outputModel.getLinkIndex(conversion.childLinkName));

        if (outputModel.addJoint(conversion.revJointName3,
                                &zRevJoint) == JOINT_INVALID_INDEX) {
            std::cerr << "[ERROR] convertSphericalJointsToThreeRevoluteJoints: Failed to add Z-axis revolute joint "
                      << conversion.revJointName3 << " for spherical joint "
                      << conversion.originalJointName << std::endl;
            return false;
        }
    }

    // Copy additional frames from original model
    for (FrameIndex frameIdx = inputModel.getNrOfLinks();
         frameIdx < static_cast<FrameIndex>(inputModel.getNrOfFrames());
         frameIdx++) {

        std::string frameName = inputModel.getFrameName(frameIdx);
        LinkIndex frameLinkIdx = inputModel.getFrameLink(frameIdx);
        std::string frameLinkName = inputModel.getLinkName(frameLinkIdx);
        Transform frameTransform = inputModel.getFrameTransform(frameIdx);

        outputModel.addAdditionalFrameToLink(frameLinkName, frameName, frameTransform);
    }

    // Copy sensors from original model
    outputModel.sensors() = inputModel.sensors();

    // Copy package directories and other model metadata
    outputModel.setPackageDirs(inputModel.getPackageDirs());

    // Copy visual and collision shapes
    for (LinkIndex linkIdx = 0; linkIdx < inputModel.getNrOfLinks(); linkIdx++) {
        LinkIndex newLinkIdx = outputModel.getLinkIndex(inputModel.getLinkName(linkIdx));

        // Copy visual shapes
        auto& originalVisualShapes = inputModel.visualSolidShapes().getLinkSolidShapes();
        auto& newVisualShapes = outputModel.visualSolidShapes().getLinkSolidShapes();

        for (const auto* shape : originalVisualShapes[linkIdx]) {
            newVisualShapes[newLinkIdx].push_back(shape->clone());
        }

        // Copy collision shapes
        auto& originalCollisionShapes = inputModel.collisionSolidShapes().getLinkSolidShapes();
        auto& newCollisionShapes = outputModel.collisionSolidShapes().getLinkSolidShapes();

        for (const auto* shape : originalCollisionShapes[linkIdx]) {
            newCollisionShapes[newLinkIdx].push_back(shape->clone());
        }
    }

    // Set the default base link to maintain model consistency
    if (inputModel.getDefaultBaseLink() != LINK_INVALID_INDEX) {
        std::string baseLinkName = inputModel.getLinkName(inputModel.getDefaultBaseLink());
        LinkIndex newBaseLinkIdx = outputModel.getLinkIndex(baseLinkName);
        if (newBaseLinkIdx != LINK_INVALID_INDEX) {
            outputModel.setDefaultBaseLink(newBaseLinkIdx);
        }
    }
    return true;
}

bool convertThreeRevoluteJointsToSphericalJoint(const Model& inputModel,
                                                Model& outputModel,
                                                double sphericalJointZeroMassTolerance,
                                                double sphericalJointOrthogonalityTolerance,
                                                double sphericalJointIntersectionTolerance) {

    std::vector<ThreeRevoluteJointPattern> detectedPatterns;
    std::set<std::string> jointsToRemove;
    std::set<std::string> linksToRemove;

    // Compute traversal for systematic link exploration
    Traversal traversal;
    if (!inputModel.computeFullTreeTraversal(traversal)) {
        outputModel = inputModel;
        return false;
    }

    if (traversal.getNrOfVisitedLinks() < 4) {
        // Need at least 4 links to form a pattern of 3 consecutive revolute joints
        outputModel = inputModel;
        return true;
    }

    // Look for patterns by examining consecutive links in traversal
    // Make sure we don't go out of bounds - we need 4 consecutive links (indices 0,1,2,3)

    for (TraversalIndex trvIdx = 0; trvIdx < traversal.getNrOfVisitedLinks(); trvIdx++) {
        // Get four consecutive links from traversal
        LinkConstPtr link0 = nullptr;
        LinkConstPtr link1 = nullptr;
        LinkConstPtr link2 = nullptr;
        LinkConstPtr link3 = nullptr;

        link3 = traversal.getLink(trvIdx);
        // Check that link3 is not the base
        if (link2 = traversal.getParentLinkFromLinkIndex(link3->getIndex()))
        {
            // Check that link2 is not the base
            if (link1 = traversal.getParentLinkFromLinkIndex(link2->getIndex()))
            {
                // Check that link1 is not the base
                link0 = traversal.getParentLinkFromLinkIndex(link1->getIndex());
            }
        }

        // Check that we have valid links
        if (!link0 || !link1 || !link2 || !link3) {
            continue;
        }

        // Check intermediate links have zero mass
        double mass1 = link1->getInertia().getMass();
        double mass2 = link2->getInertia().getMass();

        if (mass1 > sphericalJointZeroMassTolerance ||
            mass2 > sphericalJointZeroMassTolerance) {
                continue;
        }

        // Get the three joints connecting them
        IJointConstPtr joint01 = getJointFromLinks(inputModel, link0, link1);
        IJointConstPtr joint12 = getJointFromLinks(inputModel, link1, link2);
        IJointConstPtr joint23 = getJointFromLinks(inputModel, link2, link3);

        // Skip if any joint is null - this is more likely than the comment suggests
        if (!joint01 || !joint12 || !joint23) {
            continue;
        }

        // Now it's safe to use dynamic_cast
        const RevoluteJoint* revJoint01 = dynamic_cast<const RevoluteJoint*>(joint01);
        const RevoluteJoint* revJoint12 = dynamic_cast<const RevoluteJoint*>(joint12);
        const RevoluteJoint* revJoint23 = dynamic_cast<const RevoluteJoint*>(joint23);

        if (!revJoint01 || !revJoint12 || !revJoint23) {
            // One of the joints is not revolute
            continue;
        }

        // Get joint axes
        Axis axis01 = revJoint01->getAxis(link1->getIndex());
        Axis axis12 = revJoint12->getAxis(link2->getIndex());
        Axis axis23 = revJoint23->getAxis(link3->getIndex());

        // We need to express all axes in the same reference frame (let's use link0 frame)
        // Get the transforms from each joint's child link back to link0
        Transform transform01 = revJoint01->getRestTransform(link0->getIndex(), link1->getIndex());
        Transform transform02 = transform01 * revJoint12->getRestTransform(link1->getIndex(), link2->getIndex());
        Transform transform03 = transform02 * revJoint23->getRestTransform(link2->getIndex(), link3->getIndex());

        // Transform all axes to link0 frame
        Axis axis01_inLink0Frame = transform01 * axis01;
        Axis axis12_inLink0Frame = transform02 * axis12;
        Axis axis23_inLink0Frame = transform03 * axis23;

        // Check orthogonality
        if (!areVectorsOrthogonal(axis01_inLink0Frame.getDirection(), axis12_inLink0Frame.getDirection(), axis23_inLink0Frame.getDirection(),
                                 sphericalJointOrthogonalityTolerance)) {
            continue;
        }

        // Check axes intersection
        Position intersectionPoint;
        if (!findAxisIntersectionPoint(axis01_inLink0Frame, axis12_inLink0Frame, axis23_inLink0Frame, intersectionPoint,
                                      sphericalJointIntersectionTolerance)) {
            continue;
        }

        // Found valid pattern!
        ThreeRevoluteJointPattern pattern;
        pattern.parentLinkName = inputModel.getLinkName(link0->getIndex());
        pattern.intermediateLinkName1 = inputModel.getLinkName(link1->getIndex());
        pattern.intermediateLinkName2 = inputModel.getLinkName(link2->getIndex());
        pattern.childLinkName = inputModel.getLinkName(link3->getIndex());
        pattern.revJointName1 = inputModel.getJointName(joint01->getIndex());
        pattern.revJointName2 = inputModel.getJointName(joint12->getIndex());
        pattern.revJointName3 = inputModel.getJointName(joint23->getIndex());

        // Compute combined transform
        Transform transform12 = revJoint12->getRestTransform(link1->getIndex(), link2->getIndex());
        Transform transform23 = revJoint23->getRestTransform(link2->getIndex(), link3->getIndex());

        pattern.parentLink_H_childLink = transform01 * transform12 * transform23;
        pattern.jointCenter = intersectionPoint;

        detectedPatterns.push_back(pattern);

        // Mark for removal
        jointsToRemove.insert(pattern.revJointName1);
        jointsToRemove.insert(pattern.revJointName2);
        jointsToRemove.insert(pattern.revJointName3);
        linksToRemove.insert(pattern.intermediateLinkName1);
        linksToRemove.insert(pattern.intermediateLinkName2);
    }

    if (detectedPatterns.empty()) {
        outputModel = inputModel; // No patterns found
        return true;
    }

    // Create output model by copying input model structure
    outputModel = Model();
    outputModel.setPackageDirs(inputModel.getPackageDirs());

    // Copy all links except the intermediate ones to be removed
    for (LinkIndex linkIdx = 0; linkIdx < inputModel.getNrOfLinks(); linkIdx++) {
        std::string linkName = inputModel.getLinkName(linkIdx);
        if (linksToRemove.find(linkName) == linksToRemove.end()) {
            outputModel.addLink(linkName, *inputModel.getLink(linkIdx));
        }
    }

    // Copy all joints except those being replaced by spherical joints
    for (JointIndex jointIdx = 0; jointIdx < inputModel.getNrOfJoints(); jointIdx++) {
        std::string jointName = inputModel.getJointName(jointIdx);
        if (jointsToRemove.find(jointName) == jointsToRemove.end()) {
            const IJoint* joint = inputModel.getJoint(jointIdx);
            LinkIndex parentLinkIdx = joint->getFirstAttachedLink();
            LinkIndex childLinkIdx = joint->getSecondAttachedLink();

            std::string parentLinkName = inputModel.getLinkName(parentLinkIdx);
            std::string childLinkName = inputModel.getLinkName(childLinkIdx);

            // Skip if child link is one of the intermediate links being removed
            if (linksToRemove.find(childLinkName) != linksToRemove.end()) {
                continue;
            }

            outputModel.addJoint(parentLinkName, childLinkName, jointName, joint);
        }
    }

    // Add spherical joints for each detected pattern
    for (const auto& pattern : detectedPatterns) {
        // Generate spherical joint name
        std::string sphericalJointName = pattern.revJointName1;
        if (sphericalJointName.find("_x") != std::string::npos) {
            sphericalJointName = sphericalJointName.substr(0, sphericalJointName.find("_x"));
        } else if (sphericalJointName.find("_rev_") != std::string::npos) {
            sphericalJointName = sphericalJointName.substr(0, sphericalJointName.find("_rev_"));
        }

        // Create spherical joint
        SphericalJoint sphericalJoint(pattern.parentLink_H_childLink);

        // Set joint center
        LinkIndex parentLinkIdx = outputModel.getLinkIndex(pattern.parentLinkName);
        sphericalJoint.setJointCenter(parentLinkIdx, pattern.jointCenter);

        // Add joint to output model
        outputModel.addJoint(pattern.parentLinkName, pattern.childLinkName, sphericalJointName, &sphericalJoint);
    }

    // Copy additional frames, sensors, and visual/collision shapes
    for (FrameIndex frameIdx = inputModel.getNrOfLinks();
         frameIdx < static_cast<FrameIndex>(inputModel.getNrOfFrames());
         frameIdx++) {

        std::string frameName = inputModel.getFrameName(frameIdx);
        LinkIndex frameLinkIdx = inputModel.getFrameLink(frameIdx);
        std::string frameLinkName = inputModel.getLinkName(frameLinkIdx);

        // Skip frames attached to removed links
        if (linksToRemove.find(frameLinkName) != linksToRemove.end()) {
            continue;
        }

        Transform frameTransform = inputModel.getFrameTransform(frameIdx);
        outputModel.addAdditionalFrameToLink(frameLinkName, frameName, frameTransform);
    }

    // Copy sensors
    outputModel.sensors() = inputModel.sensors();

    // Copy visual and collision shapes for retained links
    for (LinkIndex linkIdx = 0; linkIdx < inputModel.getNrOfLinks(); linkIdx++) {
        std::string linkName = inputModel.getLinkName(linkIdx);
        if (linksToRemove.find(linkName) != linksToRemove.end()) {
            continue; // Skip removed links
        }

        LinkIndex newLinkIdx = outputModel.getLinkIndex(linkName);
        if (newLinkIdx == LINK_INVALID_INDEX) {
            continue;
        }

        // Copy visual shapes
        auto& originalVisualShapes = inputModel.visualSolidShapes().getLinkSolidShapes();
        auto& newVisualShapes = outputModel.visualSolidShapes().getLinkSolidShapes();

        for (const auto* shape : originalVisualShapes[linkIdx]) {
            newVisualShapes[newLinkIdx].push_back(shape->clone());
        }

        // Copy collision shapes
        auto& originalCollisionShapes = inputModel.collisionSolidShapes().getLinkSolidShapes();
        auto& newCollisionShapes = outputModel.collisionSolidShapes().getLinkSolidShapes();

        for (const auto* shape : originalCollisionShapes[linkIdx]) {
            newCollisionShapes[newLinkIdx].push_back(shape->clone());
        }
    }

    // Set default base link
    if (inputModel.getDefaultBaseLink() != LINK_INVALID_INDEX) {
        std::string baseLinkName = inputModel.getLinkName(inputModel.getDefaultBaseLink());
        if (linksToRemove.find(baseLinkName) == linksToRemove.end()) {
            LinkIndex newBaseLinkIdx = outputModel.getLinkIndex(baseLinkName);
            if (newBaseLinkIdx != LINK_INVALID_INDEX) {
                outputModel.setDefaultBaseLink(newBaseLinkIdx);
            }
        }
    }

    return true;
}

}
