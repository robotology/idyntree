/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Francesco Romano
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include "iDynTree/yarp/YARPConfigurationsLoader.h"
#include "yarp/os/Value.h"

bool iDynTree::parseRotationMatrix(const yarp::os::Searchable& rf, const std::string& key, iDynTree::Rotation& rotation)
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
            rotation.setVal(row, column, innerList->get(column).asDouble());
        }
    }
    return true;
}