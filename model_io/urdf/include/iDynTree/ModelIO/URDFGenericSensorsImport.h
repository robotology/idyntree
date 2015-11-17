/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_GENERIC_SENSORS_IMPORT_H
#define IDYNTREE_URDF_GENERIC_SENSORS_IMPORT_H

#include <string>

namespace iDynTree

{

class SensorsList;

/**
 * Load a iDynTree::SensorsList object from a URDF file.
 *
 * \note Experimental version for a more generic set of sensors 
 *  other than gazebo extension F/T.
 *
 * @return true if all went ok, false otherwise.
 */
bool genericSensorsListFromURDF(const std::string & urdf_filename,
                         iDynTree::SensorsList & output);

/**
 * Load a iDynTree::SensorsList object from a URDF string.
 *
 * \note Experimental version for a more generic set of sensors 
 *  other than gazebo extension F/T.
 *
 * @return true if all went ok, false otherwise.
 */
bool genericSensorsListFromURDFString(const std::string & urdf_string,
                               iDynTree::SensorsList & output);

}

#endif