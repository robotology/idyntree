/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_URDF_SOLID_SHAPES_IMPORT_H
#define IDYNTREE_URDF_SOLID_SHAPES_IMPORT_H

#include <string>

#include <iDynTree/Core/Utils.h>

namespace iDynTree

{

class Model;
class ModelSolidShapes;

/**
 * \ingroup ModelIO
 *
 * Parse solid shapes from an URDF file.
 *
 * @param[in] urdfGeometryType can be "visual", "collision".
 * @return true if all went well, false otherwise.
 *
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::ModelSolidShapes from a URDF file.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::ModelSolidShapes from a URDF file.")
bool solidShapesFromURDF(const std::string & urdf_filename,
                         const Model & model,
                         const std::string urdfGeometryType,
                               ModelSolidShapes & output);

/**
 * \ingroup ModelIO
 *
 * Parse solid shapes from an URDF string.
 *
 * @param[in] urdf_string
 * @param[in] urdf_filename if the urdf was loaded from a
 * @param[in] urdfGeometryType can be "visual" or "collision".
 * @return true if all went well, false otherwise.
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::ModelSolidShapes from a URDF string.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::ModelSolidShapes from a URDF string.")
bool solidShapesFromURDFString(const std::string & urdf_string,
                               const std::string & urdf_filename,
                               const Model & model,
                               const std::string urdfGeometryType,
                                     iDynTree::ModelSolidShapes & output);

}

#endif
