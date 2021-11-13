/*
 * SPDX-FileCopyrightText: 2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


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

enum ApproximateSolidShapesWithPrimitiveShapeConversionType {
    ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes
};

struct ApproximateSolidShapesWithPrimitiveShapeOptions 
{
    ApproximateSolidShapesWithPrimitiveShapeConversionType conversionType = ConvertSolidShapesWithEnclosingAxisAlignedBoundingBoxes;
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
