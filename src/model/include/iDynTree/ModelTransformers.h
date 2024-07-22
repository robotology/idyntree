// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

#include <unordered_map>
#include <string>
#include <vector>

namespace iDynTree
{
class Model;
class SensorsList;
class Traversal;

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
 * @warning This function does not handle SensorsList contained inside the input model.
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
 * This function takes in input a iDynTree::Model and
 * an ordered list of joints and returns a model with
 * just the joint specified in the list, with that exact order.
 *
 * All other joints are be removed by lumping (i.e. fusing together)
 * the inertia of the links that are connected by that joint, assuming the joint
 * to be in "rest" position (i.e. zero for revolute or prismatic joints), or the position
 * specified in the removedJointPositions if a given joint is specified in removedJointPositions
 *
 */
bool createReducedModel(const Model& fullModel,
                        const std::vector<std::string>& jointsInReducedModel,
                        Model& reducedModel,
                        const std::unordered_map<std::string, double>& removedJointPositions);

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
 * @warning This function does not handle SensorsList contained inside the input model.
 *
 * @return true if all went well, false if there was an error in conversion.
 */
bool createModelWithNormalizedJointNumbering(const Model& model,
                                             const std::string& baseForNormalizedJointNumbering,
                                             Model& reducedModel);


/**
 * Extract sub model from sub model traversal.
 *
 * This function creates a new iDynTree::Model containing links and joints composing the subModelTraversal.
 * The function takes in input a iDynTree::Model and a iDynTree::Traversal. The new model will contain joints
 * and links composing the subModelTraversal, with the same order.
 * The FT sensor frames are added as additional frames.
 *
 * @warning This function does not handle SensorsList contained inside the input model.
 *
 *
 * @return true if all went well, false if there was an error in creating the sub model.
 */
bool extractSubModel(const iDynTree::Model& fullModel, const iDynTree::Traversal& subModelTraversal,
                     iDynTree::Model& outputSubModel);

/**
 * Add automatically generated names to all visual and collision solid shapes of the model.
 *
 * This function creates a new iDynTree::Model that is identical to the input one,
 * but that sets a valid name for all the visual and collision solid shapes of the model.
 *
 * For links that already have at least either collision or visual with valid name,
 * **all** the corresponding shapes (collision or visual) will not be modified.
 *
 * For links in which all shapes of a given type that do not have a valid name, a name will
 * be generated as the following:
 *   * If there is a single collision or visual element in the link, it will be named
 *     <linkName>_collision or <linkName>_visual
 *   * If there are multiple collision or visual elements in the link, the solid shapes will be
 *     <linkName>_collision_0, <linkName>_collision_1, ... <linkName>_collision_n or
 *     <linkName>_visual_0, <linkName>_visual_1, ... <linkName>_visual_n
 *
 *
 * @return true if all went well, false if there was an error in creating the sub model.
 */
bool addValidNamesToAllSolidShapes(const iDynTree::Model& inputModel,
                                   iDynTree::Model& outputModel);

/**
 * Transform the input model in model that can be exported as URDF with the given base link.
 *
 * In iDynTree, the link frame can be placed without constraint w.r.t. to the joints to which
 * the link is connected. On the other hand, in the URDF format, the origin of the frame of the child link
 * connected to its parent with a non-fixed joint **must** lay on the axis of the joint.
 *
 * That means that if you want to export a model with an arbitrary base link, some link frame will need
 * to be moved to respect the constraint given by the URDF format. This function perform exactly this
 * transformation, ensuring that inertia, visual and collision information is probably accounted for,
 * and leaving the original link frames in the model as "additional frames" with the naming scheme
 * <linkName>_original_frame .
 *
 * Note that the operation done depends on the base link used, if you want to use a different
 * base link, change the default base link of the model via inputModel.setDefaultBaseLink method.
 *
 * @return true if all went well, false if there was an error in creating the sub model.
 */
bool moveLinkFramesToBeCompatibleWithURDFWithGivenBaseLink(const iDynTree::Model& inputModel,
                                                           iDynTree::Model& outputModel);

}


#endif
