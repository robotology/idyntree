/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_SOLID_SHAPES_IMPORT_H
#define IDYNTREE_URDF_SOLID_SHAPES_IMPORT_H

#include <string>

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
 */
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
 */
bool solidShapesFromURDFString(const std::string & urdf_string,
                              const std::string & urdf_filename,
                              const Model & model,
                              const std::string urdfGeometryType,
                                    iDynTree::ModelSolidShapes & output);

}

#endif
