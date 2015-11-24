/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Author: Naveen Kuppuswamy
 * email:  naveen.kuppuswamyt@iit.it
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
// #include "iDynTree/Sensors/IMeasurement.hpp"

#include "iDynTree/Sensors/Accelerometer.hpp"

#include "iDynTree/Core/Transform.h"

namespace iDynTree {

struct Accelerometer::AccelerometerPrivateAttributes
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


Accelerometer::Accelerometer()
{
    this->pimpl = new AccelerometerPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";

}

Accelerometer::Accelerometer(const Accelerometer& other):
    pimpl(new AccelerometerPrivateAttributes(*(other.pimpl)))
{

}

Accelerometer& Accelerometer::operator=(const Accelerometer& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


Accelerometer::~Accelerometer()
{
    delete this->pimpl;
}

bool Accelerometer::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool Accelerometer::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor) const
{
      this->pimpl->link_H_sensor = link_H_sensor;
      return true;
}


bool Accelerometer::setParent(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool Accelerometer::setParentIndex(const int &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;

}

std::string Accelerometer::getParent() const
{
    return(this->pimpl->parent_link_name);
}

int Accelerometer::getParentIndex() const
{
    return(this->pimpl->parent_link_index);
}

bool Accelerometer::isValid() const
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

Sensor* Accelerometer::clone() const
{
    return (Sensor *)new Accelerometer(*this);
}


std::string Accelerometer::getName() const
{
    return this->pimpl->name;
}

SensorType Accelerometer::getSensorType() const
{
    return ACCELEROMETER;
}

/*
 * To be implmented in future based on interface and requirements
bool getAccelerationOfLink(const iDynTree::LinAcceleration & measured_acceleration,
                           iDynTree::LinAcceleration & linear_acceleration_of_link )
{
    return(true);
}
*/
}