/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef IDYNTREE_URDF_MODEL_IMPORT_H
#define IDYNTREE_URDF_MODEL_IMPORT_H

#include <string>

namespace iDynTree

{

class Model;

/**
 * Load a iDynTree::Model object from a URDF file.
 *
 * @return true if all went ok, false otherwise.
 */
bool modelFromURDF(const std::string & urdf_filename,
                   iDynTree::Model & output);

/**
 * Load a iDynTree::Model object from a URDF string.
 *
 * @return true if all went ok, false otherwise.
 */
bool modelFromURDFString(const std::string & urdf_string,
                         iDynTree::Model & output);


}

#endif