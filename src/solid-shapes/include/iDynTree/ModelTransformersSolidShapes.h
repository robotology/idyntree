// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause



/**
 * \file ModelTransformersSolidShapes.h
 *  \brief Collection of function to modify model, related to solid shapes.
 *
*/


#ifndef IDYNTREE_MODEL_TRANSFORMERS_SOLID_SHAPES_H
#define IDYNTREE_MODEL_TRANSFORMERS_SOLID_SHAPES_H

namespace iDynTree
{
class Model;
class SensorsList;

enum class ApproximateSolidShapesWithPrimitiveShapeConversionType {
    ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes
};

/**
 * Enum class representing which shape should be approximated.
 * Namely teh visual, collision or both shapes.
 */
enum class ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate {
    VisualShapes,
    CollisionShapes,
    BothShapes
};

struct ApproximateSolidShapesWithPrimitiveShapeOptions 
{
    ApproximateSolidShapesWithPrimitiveShapeConversionType conversionType = ApproximateSolidShapesWithPrimitiveShapeConversionType::ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes;
    ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate shapesToApproximate = ApproximateSolidShapesWithPrimitiveShapeShapesToApproximate::BothShapes;
};

/**
 * \function Approximate solid shapes of the input model with some primitive shapes.
 *
 * This function takes in input a Model, and returns in output the same model, but with all 
 * solid shapes of the model (both collision and visual) substituted with a primitive shape 
 * that approximates in some way the original solid shape.
 *
 * \note At the moment, the only conversion type provided is to approximate each solid
 * shape of the model with its axis aligned bounding box, using the iDynTree::computeBoundingBoxFromShape
 * function.
 *
 */
bool approximateSolidShapesWithPrimitiveShape(const Model& inputModel,
                                              Model& outputModel,
                                              ApproximateSolidShapesWithPrimitiveShapeOptions options=ApproximateSolidShapesWithPrimitiveShapeOptions());


}

#endif
