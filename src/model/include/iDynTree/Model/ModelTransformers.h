/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

/**
 * \file ModelTransformers.h
 *  \brief Collection of function to modify model in various ways
 *
 *
 *  In this file a series of functions for transforming Model
 *   objects are provided
*/


#ifndef IDYNTREE_MODEL_TRANSFORMERS_H
#define IDYNTREE_MODEL_TRANSFORMERS_H

namespace iDynTree
{
class Model;

/**
 * \function Remove all fake links in the model, transforming them in frames.
 *
 * Given a Model in input, this function copies all its links
 * and joints to the model in output, except for links that recognized
 * as "fake links".
 *
 * The condition for a link to be classified as "fake link" are:
 *  * The link has a zero mass.
 *  * The link is a leaf, i.e. it is connected to only one neighbor.
 *  * The link is connected to its only neighbor with a fixed joint.
 *
 * Once a "fake link" has been identified to respect this two conditions,
 * it and the joint that it connects it to its only neighbor is not copied
 * to the output model, but a frame with the same name of the "fake link"
 * and with the same transform is added to the model.
 *
 * \note The definition of "fake link" used in this function excludes
 *       the case in which two fake links are attached to one another.
 *
 */
bool removeFakeLinks(const Model& modelWithFakeLinks,
                     Model& modelWithoutFakeLinks);

}

#endif