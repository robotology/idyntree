/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
class SensorsList;

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

/**
 * This function takes in input a iDynTree::Model and
 * an ordered list of joints and returns a model with
 * just the joint specified in the list, with that exact order.
 *
 * All other joints are be removed by lumping (i.e. fusing together)
 * the inertia of the links that are connected by that joint, assuming the joint
 * to be in "rest" position (i.e. zero for revolute or prismatic joints). The links eliminated
 * with this process are be added back to the reduced model as "frames",
 * and are copied in the same way all the additional frames of the lumped links.
 *
 */
bool createReducedModel(const Model& fullModel,
                        const std::vector<std::string>& jointsInReducedModel,
                        Model& reducedModel);

/**
 * @brief Given a specified base, return a model with a "normalized" joint numbering for that base.
 *
 * This function takes in input a iDynTree::Model and a name of a link in that model.
 * It returns a model identical to the one in input, but with the joint serialization
 * of the non-fixed joint modified in such a way that a non-fixed joint has an index higher than
 * all the non-fixed joints on the path between it and the base. After all the non-fixed joints,
 * the fixed joints are also added with the same criteria, but applied to fixed joints.
 *
 * @note This method make sure that the non-fixed joint in the model have a "regular numbering"
 *       as described in Featherstone "Rigid Body Dynamics Algorithm", section 4.1.2 . Note that
 *       this numbering is not required by any algorithm in iDynTree, but it may be useful for
 *       example to ensure that for a chain model the joint numbering is the one induced by the
 *       kinematic structure.
 *
 * @return true if all went well, false if there was an error in conversion.
 */
bool createModelWithNormalizedJointNumbering(const Model& model,
                                             const std::string& baseForNormalizedJointNumbering,
                                             Model& reducedModel);


}

#endif
