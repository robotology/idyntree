// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelTransformers.h>

#include <iDynTree/Model.h>
#include <iDynTree/Link.h>

#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelTestUtils.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelExporter.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace iDynTree;

double random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int real_random_int(int initialValue, int finalValue)
{
    int length = finalValue - initialValue;
    return initialValue + rand() % length;
}

void setRandomState(iDynTree::KinDynComputations & originalKinDyn, iDynTree::KinDynComputations & transformedKinDyn)
{
    size_t dofs = originalKinDyn.getNrOfDegreesOfFreedom();
    Transform    worldTbase;
    Twist        baseVel;
    Vector3 gravity;

    iDynTree::VectorDynSize qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
            Position(random_double(),random_double(),random_double()));

    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = real_random_double();
    }

    for(size_t dof=0; dof < dofs; dof++)

    {
        qj(dof) = random_double();
        dqj(dof) = random_double();
        ddqj(dof) = random_double();
    }

    ASSERT_IS_TRUE(originalKinDyn.setRobotState(worldTbase,qj,baseVel,dqj,gravity));
    ASSERT_IS_TRUE(transformedKinDyn.setRobotState(worldTbase,qj,baseVel,dqj,gravity));
}


// This test ensures that moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink transform the
// models in a way that does not modify its inertial, visual and collision properties
void checkThatMoveLinkFramesToBeCompatibleWithURDFWithGivenBaseLinkIsConsistent()
{

    for (size_t i=0; i < 10; i++)
    {
        // Generate random model
        size_t nrOfJoints = 20;
        size_t nrOFAdditionalFrames = 10;

        iDynTree::Model originalModel = iDynTree::getRandomModel(nrOfJoints, nrOFAdditionalFrames);

        // Get random base link
        iDynTree::LinkIndex originalBaseLinkIndex = static_cast<iDynTree::LinkIndex>(rand() % originalModel.getNrOfLinks());
        originalModel.setDefaultBaseLink(originalBaseLinkIndex);
        std::string baseLink = originalModel.getLinkName(originalBaseLinkIndex);

        iDynTree::Model transformedModel;
        ASSERT_IS_TRUE(moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink(originalModel, transformedModel));

        // Check that the default base link is propagated
        ASSERT_IS_TRUE(originalModel.getLinkName(originalModel.getDefaultBaseLink()) == transformedModel.getLinkName(transformedModel.getDefaultBaseLink()));

        // Check consistency
        ASSERT_IS_TRUE(originalModel.getNrOfJoints() == transformedModel.getNrOfJoints());
        ASSERT_IS_TRUE(originalModel.getNrOfLinks() == transformedModel.getNrOfLinks());
        // The transformed model has more frames, as it stores the original_frames as <linkName>_original_frame
        ASSERT_IS_TRUE(originalModel.getNrOfFrames() <= transformedModel.getNrOfFrames());

        // Check that joint serialization remains the same
        for(iDynTree::JointIndex jntIdx=0; jntIdx < originalModel.getNrOfJoints(); jntIdx++)
        {
            ASSERT_IS_TRUE(originalModel.getJointName(jntIdx) == transformedModel.getJointName(jntIdx));
        }

        // Create a kindyn object for each model, and assign a random state
        iDynTree::KinDynComputations originalKinDyn;
        ASSERT_IS_TRUE(originalKinDyn.loadRobotModel(originalModel));
        ASSERT_IS_TRUE(originalKinDyn.setFloatingBase(baseLink));
        iDynTree::KinDynComputations transformedKinDyn;
        ASSERT_IS_TRUE(transformedKinDyn.loadRobotModel(transformedModel));
        ASSERT_IS_TRUE(transformedKinDyn.setFloatingBase(baseLink));
        setRandomState(originalKinDyn, transformedKinDyn);

        // Check kinematics: make sure that the base link did not moved and the world_H_base is the same for both models
        ASSERT_EQUAL_TRANSFORM(originalKinDyn.getWorldTransform(originalModel.getLinkIndex(baseLink)),
                               transformedKinDyn.getWorldTransform(transformedModel.getLinkIndex(baseLink)));

        // Check kinematics: make sure that the kinematics between additional frames remain consistent
        // (as only the link frame can be moved by the function)
        std::string firstAdditionalFrame = originalModel.getFrameName(originalModel.getNrOfLinks());
        for(iDynTree::FrameIndex frameIndex = originalModel.getNrOfLinks(); frameIndex < originalModel.getNrOfFrames(); frameIndex++)
        {
            std::string secondAdditionalFrame = originalModel.getFrameName(frameIndex);

            ASSERT_EQUAL_TRANSFORM(originalKinDyn.getRelativeTransform(firstAdditionalFrame, secondAdditionalFrame),
                                   transformedKinDyn.getRelativeTransform(firstAdditionalFrame, secondAdditionalFrame));

            // As the base link can't be moved, we also check that the world transform of additional frames is consistent
            ASSERT_EQUAL_TRANSFORM(originalKinDyn.getWorldTransform(secondAdditionalFrame),
                                   transformedKinDyn.getWorldTransform(secondAdditionalFrame));
        }

        // Check inertia: make sure that the inertia of each link (projected back in world) is the same
        for(iDynTree::LinkIndex lnkIdx=0; lnkIdx < originalModel.getNrOfLinks(); lnkIdx++)
        {
            std::string linkName = originalModel.getLinkName(lnkIdx);
            iDynTree::SpatialInertia originalLinkInertia = originalKinDyn.getWorldTransform(originalModel.getLinkIndex(linkName))*(originalModel.getLink(originalModel.getLinkIndex(linkName))->getInertia());
            iDynTree::SpatialInertia transformedLinkInertia = transformedKinDyn.getWorldTransform(transformedModel.getLinkIndex(linkName))*(transformedModel.getLink(transformedModel.getLinkIndex(linkName))->getInertia());
            ASSERT_EQUAL_MATRIX(originalLinkInertia.asMatrix(), transformedLinkInertia.asMatrix());
        }

        // Check inertia: make sure that COM is the same
        iDynTree::Position originalCOM = originalKinDyn.getCenterOfMassPosition();
        iDynTree::Position transformedCOM = transformedKinDyn.getCenterOfMassPosition();

        ASSERT_EQUAL_VECTOR(originalCOM, transformedCOM);

        // Check inertia: make sure that mass matrix is the same
        iDynTree::FreeFloatingMassMatrix originalMassMatrix(originalModel);
        iDynTree::FreeFloatingMassMatrix transformedMassMatrix(transformedModel);
        ASSERT_IS_TRUE(originalKinDyn.getFreeFloatingMassMatrix(originalMassMatrix));
        ASSERT_IS_TRUE(transformedKinDyn.getFreeFloatingMassMatrix(transformedMassMatrix));
        ASSERT_EQUAL_MATRIX(originalMassMatrix, transformedMassMatrix);

        // Check collision and visual: check that transform between the visual and collision element w.r.t. to a given additional
        // frame is always the same

        for(iDynTree::LinkIndex lnkIdx=0; lnkIdx < originalModel.getNrOfLinks(); lnkIdx++)
        {
            auto& originalVisual = originalModel.visualSolidShapes().getLinkSolidShapes();
            auto& transformedVisual = transformedModel.visualSolidShapes().getLinkSolidShapes();
            for(int shapeIdx=0; shapeIdx < originalVisual[lnkIdx].size(); shapeIdx++)
            {
                // Check that link index remains constant
                ASSERT_IS_TRUE(originalModel.getLinkName(lnkIdx) == transformedModel.getLinkName(lnkIdx));

                // Get firstAdditionalFrame_H_visual in original model and transformed model
                iDynTree::Transform original_frame_H_visual =
                    originalKinDyn.getRelativeTransform(firstAdditionalFrame, originalModel.getLinkName(lnkIdx))*
                    originalVisual[lnkIdx][shapeIdx]->getLink_H_geometry();

                iDynTree::Transform transformed_frame_H_visual =
                    transformedKinDyn.getRelativeTransform(firstAdditionalFrame, transformedModel.getLinkName(lnkIdx))*
                    transformedVisual[lnkIdx][shapeIdx]->getLink_H_geometry();

                ASSERT_EQUAL_TRANSFORM(original_frame_H_visual, transformed_frame_H_visual);
            }

            auto& originalCollision = originalModel.collisionSolidShapes().getLinkSolidShapes();
            auto& transformedCollision = transformedModel.collisionSolidShapes().getLinkSolidShapes();
            for(int shapeIdx=0; shapeIdx < originalCollision[lnkIdx].size(); shapeIdx++)
            {
                // Check that link index remains constant
                ASSERT_IS_TRUE(originalModel.getLinkName(lnkIdx) == transformedModel.getLinkName(lnkIdx));

                // Get firstAdditionalFrame_H_collision in original model and transformed model
                iDynTree::Transform original_frame_H_visual =
                    originalKinDyn.getRelativeTransform(firstAdditionalFrame, originalModel.getLinkName(lnkIdx))*
                    originalCollision[lnkIdx][shapeIdx]->getLink_H_geometry();

                iDynTree::Transform transformed_frame_H_visual =
                    transformedKinDyn.getRelativeTransform(firstAdditionalFrame, transformedModel.getLinkName(lnkIdx))*
                    transformedCollision[lnkIdx][shapeIdx]->getLink_H_geometry();

                ASSERT_EQUAL_TRANSFORM(original_frame_H_visual, transformed_frame_H_visual);
            }
        }

        // Verify that the transformed model can be exported
        iDynTree::ModelExporter transformedModelExporter;
        transformedModelExporter.init(transformedModel);
        std::string transformedURDFStringExported;
        ASSERT_IS_TRUE(transformedModelExporter.exportModelToString(transformedURDFStringExported));
    }
}

int main()
{
    checkThatMoveLinkFramesToBeCompatibleWithURDFWithGivenBaseLinkIsConsistent();

    return EXIT_SUCCESS;
}