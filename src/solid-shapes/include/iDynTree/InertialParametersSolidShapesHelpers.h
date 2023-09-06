// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H
#define IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H

#include <iDynTree/Model.h>
#include <iDynTree/SolidShapes.h>

namespace iDynTree
{

/**
 * @brief Estimate the inertial parameters of a model using link bounding boxes and the total mass.
 *
 * @param[in] totalMass The total mass of the model, in Kilograms.
 * @param[in] model The model, used to extract the bounding box of the links.
 * @param[out] inertialParameters vector of inertial parameters, in the format used by Model::getInertialParameters
 * @return true if all went well, false otherwise .
 *
 * @note if inertialParameters does not have the correct size, it will be resized.
 * @warning This function needs to allocate some internal buffers, and so it performs dynamic memory allocation.
 *
 * @note This function requires IDYNTREE_USES_ASSIMP to be true, otherwise it always returns false.
 */
bool estimateInertialParametersFromLinkBoundingBoxesAndTotalMass(const double totalMass,
                                                                 iDynTree::Model& model,
                                                                 VectorDynSize& estimatedInertialParams);
/**
 * @brief Compute bounding box from a solid shape object
 *
 * @param[in] geom The link collision as a iDynTree solid shape object
 * @param[out] box The bounding box for the solid shape object
 * @return true if all went well, false otherwise .
 *
 * @note If the shape is an ExternalMesh type, this function requires IDYNTREE_USES_ASSIMP to be true, otherwise it always returns false.
 * @note If the shape is an ExternalMesh type, an Axis-Aligned Bounding Box (AABB) is computed from the mesh vertices. 
 *       This means that the returned bounding box is neither unique nor a minimum volume bounding box for the given mesh.
 *       The axes alignment to compute AABB is done with respect to the frame associated with the shape.
 */
bool computeBoundingBoxFromShape(const SolidShape& geom, Box& box);

/**
 * @brief Get the bounding box vertices in the link frame
 *
 * @param[in] box The link collision solid shape as a Box object.
 * @return vector of vertex positions in the link frame.
 */
std::vector<Position> computeBoxVertices(const Box box);
}

#endif /* IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H */
