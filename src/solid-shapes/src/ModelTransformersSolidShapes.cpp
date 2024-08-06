// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/ModelTransformersSolidShapes.h>

#include <iDynTree/Model.h>
#include <iDynTree/InertialParametersSolidShapesHelpers.h>

#include <cassert>
#include <set>

namespace iDynTree
{

bool approximateSolidShapesWithBoundingBoxes(const iDynTree::ModelSolidShapes& inputSolidShapes,
                                         iDynTree::ModelSolidShapes& outputSolidShapes)
{
    bool retValue = true;
    outputSolidShapes.clear();
    outputSolidShapes.resize(inputSolidShapes.getLinkSolidShapes().size());
    auto& inputLinkSolidShapes = inputSolidShapes.getLinkSolidShapes();
    auto& outputLinkSolidShapes = outputSolidShapes.getLinkSolidShapes();
    for (size_t link = 0; link < inputLinkSolidShapes.size(); link++)
    {
        outputLinkSolidShapes[link].resize(inputLinkSolidShapes[link].size());
        for (size_t geom = 0; geom < inputLinkSolidShapes[link].size(); geom++)
        {
            SolidShape* inputSolidShape = inputLinkSolidShapes[link][geom];
            Box outputBox;
            bool approximationOk = computeBoundingBoxFromShape(*inputSolidShape, outputBox);
            outputLinkSolidShapes[link][geom] = outputBox.clone();
            retValue = retValue && approximationOk;
        }
    }

    return retValue;
}

bool approximateSolidShapesWithPrimitiveShape(const Model& inputModel,
                                              Model& outputModel,
                                              ApproximateSolidShapesWithPrimitiveShapeOptions options)
{
    bool retValue = true;

    // The output model is copied from the input one
    outputModel = inputModel;

    // For now the only supported option is ConvertSolidShapesWithEnclosingAxiAlignedBoundingBox,
    // that approximates the solid shapes via the iDynTree::computeBoundingBoxFromShape function

    // Approximate visual shapes
    if (options.shapesToApproximate == ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::VisualShapes ||
       options.shapesToApproximate == ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::BothShapes)
    {
        retValue = retValue && approximateSolidShapesWithBoundingBoxes(inputModel.visualSolidShapes(), outputModel.visualSolidShapes());
    }

    if (options.shapesToApproximate == ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::CollisionShapes ||
        options.shapesToApproximate == ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::BothShapes)
    {
        retValue = retValue && approximateSolidShapesWithBoundingBoxes(inputModel.collisionSolidShapes(), outputModel.collisionSolidShapes());
    }

    return retValue;
}


}
