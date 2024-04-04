// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelTransformers.h>

#include <iDynTree/Model.h>
#include <iDynTree/Link.h>

#include <iDynTree/TestUtils.h>

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


int main()
{
    checkThatOneSphereGetsAName();

    return EXIT_SUCCESS;
}