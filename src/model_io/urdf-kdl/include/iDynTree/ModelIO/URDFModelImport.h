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
 * \ingroup iDynTreeModelIO
 *
 * Load a iDynTree::Model object from a URDF file.
 *
 * Loading the URDF with this function, the  "fake links"
 * present in the URDF file are   removed from the models
 * and added back as frames. See the removeFakeLinks
 * function, that is called inside this function before
 * returning the model.
 *
 * @return true if all went ok, false otherwise.
 */
bool modelFromURDF(const std::string & urdf_filename,
                   iDynTree::Model & output);

/**
 * \ingroup iDynTreeModelIO
 *
 * Loading the URDF with this function, the  "fake links"
 * present in the URDF file are   removed from the models
 * and added back as frames. See the removeFakeLinks
 * function, that is called inside this function before
 * returning the model. .
 *
 * @return true if all went ok, false otherwise.
 */
bool modelFromURDFString(const std::string & urdf_string,
                         iDynTree::Model & output);


}

#endif