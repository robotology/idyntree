/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_URDF_DOFS_IMPORT_H
#define IDYNTREE_URDF_DOFS_IMPORT_H

#include <vector>
#include <string>

namespace iDynTree

{

/**
 * \ingroup iDynTreeModelIO
 *
 * Load a list of dofs names from a URDF file.
 *
 * @return true if all went ok, false otherwise.
 *
 */
bool dofsListFromURDF(const std::string & urdf_filename,
                      std::vector<std::string>& dofs);

/**
 * \ingroup iDynTreeModelIO
 *
 * Load a list of dofs object from a URDF string.
 *
 * @return true if all went ok, false otherwise.
 */
bool dofsListFromURDFString(const std::string & urdf_string,
                            std::vector<std::string>& dofs);

}

#endif
