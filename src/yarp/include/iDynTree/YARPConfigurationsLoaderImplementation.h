// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_YARP_CONFIGURATIONS_LOADER_IMPLEMENTATION_H
#define IDYNTREE_YARP_CONFIGURATIONS_LOADER_IMPLEMENTATION_H

#include <yarp/os/Value.h>

inline bool iDynTree::parseRotationMatrix(const yarp::os::Searchable& rf, const std::string& key, iDynTree::Rotation& rotation)
{
    yarp::os::Value ini = rf.find(key);
    if (ini.isNull() || !ini.isList())
    {
        return false;
    }
    yarp::os::Bottle *outerList = ini.asList();
    if (!outerList || outerList->size() != 3)
    {
        return false;
    }
    for (int row = 0; row < outerList->size(); ++row)
    {
        yarp::os::Value& innerValue = outerList->get(row);
        if (innerValue.isNull() || !innerValue.isList())
        {
            return false;
        }
        yarp::os::Bottle *innerList = innerValue.asList();
        if (!innerList || innerList->size() != 3)
        {
            return false;
        }
        for (int column = 0; column < innerList->size(); ++column)
        {
            rotation.setVal(row, column, innerList->get(column).asFloat64());
        }
    }
    return true;
}

#endif
