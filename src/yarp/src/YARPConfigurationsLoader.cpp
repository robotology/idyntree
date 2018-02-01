/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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