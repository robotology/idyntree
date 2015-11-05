/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_SENSORS_IMPORT_H
#define IDYNTREE_URDF_SENSORS_IMPORT_H

#include <string>

namespace iDynTree

{

class SensorsList;

/**
 * Load a iDynTree::SensorsList object from a URDF file.
 *
 * \note At the moment iDynTree supports just the loading
 *       of Six Axis FT sensors information from URDF.
 *       As no definitions for sensors information are
 *       existing in the URDF specs, we load FT sensors
 *       informations using the gazebo extentions format.
 *       Please check the ftSensorsFromUrdfString implementation
 *       for more information.
 *
 * @return true if all went ok, false otherwise.
 */
bool sensorsListFromURDF(const std::string & urdf_filename,
                         iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF string.
 *
 * \note At the moment iDynTree supports just the loading
 *       of Six Axis FT sensors information from URDF.
 *       As no definitions for sensors information are
 *       existing in the URDF specs, we load FT sensors
 *       informations using the gazebo extentions format.
 *       Please check the ftSensorsFromUrdfString implementation
 *       for more information.
 *
 * @return true if all went ok, false otherwise.
 */
bool sensorsListFromURDFString(const std::string & urdf_string,
                               iDynTree::SensorsList & output);


}

#endif