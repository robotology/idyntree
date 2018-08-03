/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_URDF_GENERIC_SENSORS_IMPORT_H
#define IDYNTREE_URDF_GENERIC_SENSORS_IMPORT_H

#include <string>

#include <iDynTree/Core/Utils.h>

#ifdef __DEPRECATED
  #warning <iDynTree/ModelIO/URDFGenericSensorsImport.h> is deprecated. Please use <iDynTree/ModelIO/ModelLoader.h> and the classes contained in it. To disable this warning use -Wno-deprecated.
#endif

namespace iDynTree

{

class SensorsList;
class Model;

/**
 * Load a iDynTree::SensorsList object from a URDF file.
 *
 * The URDF syntax and the sensor supported are documented in
 * https://github.com/robotology/idyntree/blob/master/doc/model_loading.md .
 *
 *
 * @return true if all went ok, false otherwise.
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::SensorsList from a URDF file.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::SensorsList from a URDF file.")
bool sensorsFromURDF(const std::string & urdf_filename,
                     iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF file, using a previous loaded
 * iDynTree::Model from the same URDF.
 *
 * The URDF syntax and the sensor supported are documented in
 * https://github.com/robotology/idyntree/blob/master/doc/model_loading.md .
 *
 *
 * @return true if all went ok, false otherwise.
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::SensorsList from a URDF file.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromFile to load a iDynTree::SensorsList from a URDF file.")
bool sensorsFromURDF(const std::string & urdf_filename,
                     const Model & model,
                     iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF string.
 *
 * The URDF syntax and the sensor supported are documented in
 * https://github.com/robotology/idyntree/blob/master/doc/model_loading.md .
 *
 * @return true if all went ok, false otherwise.
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::SensorsList from a URDF string.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::SensorsList from a URDF string.")
bool sensorsFromURDFString(const std::string & urdf_string,
                           iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF string, using a previous loaded
 * iDynTree::Model from the same URDF.
 *
 * The URDF syntax and the sensor supported are documented in
 * https://github.com/robotology/idyntree/blob/master/doc/model_loading.md .
 *
 * @return true if all went ok, false otherwise.
 * @deprecated Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::SensorsList from a URDF string.
 */
IDYNTREE_DEPRECATED_WITH_MSG("Please use iDynTree::ModelLoader::loadModelFromString to load a iDynTree::SensorsList from a URDF string.")
bool sensorsFromURDFString(const std::string & urdf_string,
                           const Model & model,
                           iDynTree::SensorsList & output);

}

#endif
