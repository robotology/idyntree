/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H
#define IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H

#include <iDynTree/Model/Model.h>

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


}

#endif /* IDYNTREE_INERTIAL_PARAMETERS_HELPERS_H */
