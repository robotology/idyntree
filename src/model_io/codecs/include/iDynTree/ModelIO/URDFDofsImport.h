// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
