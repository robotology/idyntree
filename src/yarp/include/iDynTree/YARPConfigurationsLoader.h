// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_YARP_CONFIGURATIONS_LOADER_H
#define IDYNTREE_YARP_CONFIGURATIONS_LOADER_H

#include <iDynTree/Rotation.h>
#include <string>
#include <yarp/os/Searchable.h>

namespace iDynTree
{
/**
 * Takes a rotation matrix from configuration file.
 * Notice, the matrix is parsed row-wise.
 * @param rf yarp::os::Searchable The input searchable
 * @param key The name corresponding to the matrix to be read
 * @param rotation The output rotation
 * @return true if successfull
 */
bool parseRotationMatrix(const yarp::os::Searchable& rf,
                         const std::string& key,
                         iDynTree::Rotation& rotation);
} // namespace iDynTree

#include "YARPConfigurationsLoaderImplementation.h"

#endif