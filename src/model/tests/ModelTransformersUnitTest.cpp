// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelTransformers.h>

#include <iDynTree/Model.h>
#include <iDynTree/Link.h>

#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelTestUtils.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace iDynTree;

void checkThatOneSphereGetsAName()
{
    // Create a model with one shape with one sphere without name
    // and check that a name is correctly created
    iDynTree::Model oneSphereModel;
    iDynTree::Link oneSphereLink;
    oneSphereModel.addLink("link0", oneSphereLink);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0].resize(1);
    iDynTree::Sphere oneSphere;
    oneSphere.setLink_H_geometry(Transform::Identity());
    oneSphere.setRadius(1.0);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0][0] = new iDynTree::Sphere(oneSphere);

    // Compute the model does not have a valid name
    ASSERT_IS_TRUE(!oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0][0]->isNameValid());

    // Create name with valid names
    iDynTree::Model oneSphereModelWithValidName;

    // Assert that name are valid
    ASSERT_IS_TRUE(addValidNamesToAllSolidShapes(oneSphereModel, oneSphereModelWithValidName));

    // Check if the resulting name is valid
    ASSERT_IS_TRUE(oneSphereModelWithValidName.collisionSolidShapes().getLinkSolidShapes()[0][0]->isNameValid());
    ASSERT_IS_TRUE(oneSphereModelWithValidName.collisionSolidShapes().getLinkSolidShapes()[0][0]->getName() == "link0_collision");
}

void checkRemoveAdditionalFramesFromModel()
{
    // Create random model with 10 links and 10 additional frames
    iDynTree::Model modelWithAllAdditionalFrames = getRandomModel(10, 10, SIMPLE_JOINT_TYPES | JOINT_REVOLUTE_SO2);

    // Create an allow list of three additional frames
    std::vector<std::string> allowedAdditionalFrames;
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 7));
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 1));
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 3));

    // Create a model with only the allowed additional frames
    iDynTree::Model modelWithOnlyAllowedAdditionalFrames;
    ASSERT_IS_TRUE(removeAdditionalFramesFromModel(modelWithAllAdditionalFrames, modelWithOnlyAllowedAdditionalFrames, allowedAdditionalFrames));

    // Check that the model with only the allowed additional frames has the correct number of links and additional frames
    ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.getNrOfLinks() == modelWithAllAdditionalFrames.getNrOfLinks());
    ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.getNrOfFrames() == modelWithOnlyAllowedAdditionalFrames.getNrOfLinks() + allowedAdditionalFrames.size());

    // Check that the additional frames contained in the modelWithOnlyAllowedAdditionalFrames are the one specified in modelWithOnlyAllowedAdditionalFrames
    for (size_t i = 0; i < allowedAdditionalFrames.size(); i++)
    {
        ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.isFrameNameUsed(allowedAdditionalFrames[i]));
    }
}


int main()
{
    checkThatOneSphereGetsAName();
    checkRemoveAdditionalFramesFromModel();

    return EXIT_SUCCESS;
}
