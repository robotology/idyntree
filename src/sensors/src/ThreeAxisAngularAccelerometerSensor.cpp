/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


# include "iDynTree/Core/LinearMotionVector3.h"

#include "iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h"

#include "iDynTree/Core/Transform.h"

#include "iDynTree/Core/SpatialAcc.h"
#include "iDynTree/Core/Twist.h"

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {

struct ThreeAxisAngularAccelerometerSensor::ThreeAxisAngularAccelerometerPrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Transform from the link to the sensor
    Transform link_H_sensor;
    // Index of the parent junction
    int parent_link_index;
    // Name of the parent junction
     std::string parent_link_name;
    // Name of the link to which the Accelerometer is connected
};


ThreeAxisAngularAccelerometerSensor::ThreeAxisAngularAccelerometerSensor()
{
    this->pimpl = new ThreeAxisAngularAccelerometerPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";

}

ThreeAxisAngularAccelerometerSensor::ThreeAxisAngularAccelerometerSensor(const ThreeAxisAngularAccelerometerSensor& other):
    pimpl(new ThreeAxisAngularAccelerometerPrivateAttributes(*(other.pimpl)))
{

}

ThreeAxisAngularAccelerometerSensor& ThreeAxisAngularAccelerometerSensor::operator=(const ThreeAxisAngularAccelerometerSensor& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


ThreeAxisAngularAccelerometerSensor::~ThreeAxisAngularAccelerometerSensor()
{
    delete this->pimpl;
}

bool ThreeAxisAngularAccelerometerSensor::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool ThreeAxisAngularAccelerometerSensor::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor)
{
      this->pimpl->link_H_sensor = link_H_sensor;
      return true;
}


bool ThreeAxisAngularAccelerometerSensor::setParentLink(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool ThreeAxisAngularAccelerometerSensor::setParentLinkIndex(const LinkIndex &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;

}

std::string ThreeAxisAngularAccelerometerSensor::getParentLink() const
{
    return(this->pimpl->parent_link_name);
}

int ThreeAxisAngularAccelerometerSensor::getParentLinkIndex() const
{
    return(this->pimpl->parent_link_index);
}

bool ThreeAxisAngularAccelerometerSensor::isValid() const
{
    if( this->getName() == "" )
    {
        return false;
    }

    if( this->pimpl->parent_link_index < 0 )
    {
        // Return false because the links is not appropriately setted
        return false;
    }

    return true;
}

Sensor* ThreeAxisAngularAccelerometerSensor::clone() const
{
    return (Sensor *)new ThreeAxisAngularAccelerometerSensor(*this);
}

bool ThreeAxisAngularAccelerometerSensor::updateIndices(const Model& model)
{
    iDynTree::LinkIndex linkNewIndex = model.getLinkIndex(this->pimpl->parent_link_name);

    if( linkNewIndex == iDynTree::LINK_INVALID_INDEX )
    {
        return false;
    }

    this->pimpl->parent_link_index = linkNewIndex;

    return true;
}

std::string ThreeAxisAngularAccelerometerSensor::getName() const
{
    return this->pimpl->name;
}

SensorType ThreeAxisAngularAccelerometerSensor::getSensorType() const
{
    return THREE_AXIS_ANGULAR_ACCELEROMETER;
}

Transform ThreeAxisAngularAccelerometerSensor::getLinkSensorTransform() const
{
    return(this->pimpl->link_H_sensor);
}

Vector3 ThreeAxisAngularAccelerometerSensor::predictMeasurement(const SpatialAcc& linkAcc, const iDynTree::Twist& )
{
    Vector3 returnAcc;
    returnAcc.zero();
    if( this->pimpl->parent_link_index >= 0)
    {
        toEigen(returnAcc) = toEigen(this->pimpl->link_H_sensor.getRotation().inverse()) * toEigen(linkAcc.getAngularVec3());
    }

    return returnAcc;
}

}
