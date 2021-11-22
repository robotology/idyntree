/*
 * SPDX-FileCopyrightText: 2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <iDynTree/Core/TestUtils.h>


#include <iDynTree/ModelTransformersSolidShapes.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/Model.h>


using namespace iDynTree;

void checkThatOneSphereIsApproximatedToABox()
{
    // Create a model with one shape with one sphere, and 
    // convert it with approximateSolidShapesWithPrimitiveShape
    // with type ConvertSolidShapesWithEnclosingAxiAlignedBoundingBox, 
    // and verify that the sphere has ben substituted with a box

    iDynTree::Model oneSphereModel;
    iDynTree::Link oneSphereLink;
    oneSphereModel.addLink("link0", oneSphereLink);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0].resize(1);
    iDynTree::Sphere oneSphere;
    oneSphere.setLink_H_geometry(Transform::Identity());
    oneSphere.setRadius(1.0);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0][0] = new iDynTree::Sphere(oneSphere);

    // Compute the simplified model
    iDynTree::Model oneBoxModel;
    ApproximateSolidShapesWithPrimitiveShapeOptions options = ApproximateSolidShapesWithPrimitiveShapeOptions();
    options.conversionType = ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes;
    bool ok = approximateSolidShapesWithPrimitiveShape(oneSphereModel, oneBoxModel);
    ASSERT_IS_TRUE(ok);

    // The size of the box should be equal to the diameter of the sphere in all directions
    ASSERT_IS_TRUE(oneBoxModel.collisionSolidShapes().getLinkSolidShapes().size() == 
                   oneSphereModel.collisionSolidShapes().getLinkSolidShapes().size());
    ASSERT_IS_TRUE(oneBoxModel.collisionSolidShapes().getLinkSolidShapes()[0].size() == 
                   oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0].size());
    ASSERT_IS_TRUE(oneBoxModel.collisionSolidShapes().getLinkSolidShapes()[0][0]->isBox());
    iDynTree::Box* oneBoxPtr = oneBoxModel.collisionSolidShapes().getLinkSolidShapes()[0][0]->asBox();
    ASSERT_IS_TRUE(oneBoxPtr);
    ASSERT_EQUAL_DOUBLE(oneBoxPtr->getX(), 2*oneSphere.getRadius());
    ASSERT_EQUAL_DOUBLE(oneBoxPtr->getY(), 2*oneSphere.getRadius());
    ASSERT_EQUAL_DOUBLE(oneBoxPtr->getZ(), 2*oneSphere.getRadius());
}


int main()
{
    checkThatOneSphereIsApproximatedToABox();

    return EXIT_SUCCESS;
}

