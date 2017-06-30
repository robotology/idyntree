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


#endif