/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_URDF_SENSORS_IMPORT_H
#define IDYNTREE_URDF_SENSORS_IMPORT_H

#include <string>
#include <iDynTree/Sensors/Sensors.h>

namespace iDynTree

{

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
