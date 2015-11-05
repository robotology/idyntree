/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
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
 * \note This implements a preliminary version of sensor list
 *
 * @return true if all went ok, false otherwise.
 */
bool genericSensorsListFromURDF(const std::string & urdf_filename,
                         iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF string.
 *
 * \note This implements a preliminary version of sensor list
 *
 * @return true if all went ok, false otherwise.
 */
bool genericSensorsListFromURDFString(const std::string & urdf_string,
                               iDynTree::SensorsList & output);

}

#endif