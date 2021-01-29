/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_YARP_CONFIGURATIONS_LOADER_H
#define IDYNTREE_YARP_CONFIGURATIONS_LOADER_H

#include <yarp/os/Searchable.h>
#include <iDynTree/Core/Rotation.h>
#include <string>

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
    bool parseRotationMatrix(const yarp::os::Searchable& rf, const std::string& key, iDynTree::Rotation& rotation);
}

#include "YARPConfigurationsLoaderImplementation.h"

#endif