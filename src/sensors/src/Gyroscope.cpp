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

#include "iDynTree/Core/AngularMotionVector3.h"

#include "iDynTree/Sensors/Gyroscope.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Core/Twist.h"


namespace iDynTree {

struct Gyroscope::GyroscopePrivateAttributes
{
    // Name/id of the sensor
    std::string name;
   // Transform from the link to the sensor
    Transform link_H_sensor;
    // Index of the parent link
     int parent_link_index;
    // Name of the parent link
     std::string parent_link_name; 
};


Gyroscope::Gyroscope()
{
    this->pimpl = new GyroscopePrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";
}

Gyroscope::Gyroscope(const Gyroscope& other):
    pimpl(new GyroscopePrivateAttributes(*(other.pimpl)))
{

}

Gyroscope& Gyroscope::operator=(const Gyroscope& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


Gyroscope::~Gyroscope()
{
    delete this->pimpl;
}

bool Gyroscope::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool Gyroscope::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link_H_sensor = link_H_sensor;
    return true;
}

bool Gyroscope::setParent(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool Gyroscope::setParentIndex(const int &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;
}

bool Gyroscope::isValid() const
{
    if( this->getName() == "" )
    {
        return false;
    }

    if( this->pimpl->parent_link_index< 0 )
    {
        // Return false because the links is not appropriately setted
        return false;
    }


    return true;
}

Sensor* Gyroscope::clone() const
{
    return (Sensor *)new Gyroscope(*this);
}


std::string Gyroscope::getName() const
{
    return this->pimpl->name;
}

SensorType Gyroscope::getSensorType() const
{
    return GYROSCOPE;
}


std::string Gyroscope::getParent() const
{
    return this->pimpl->parent_link_name;
}

int Gyroscope::getParentIndex() const
{
    return this->pimpl->parent_link_index;
}


bool Gyroscope::getLinkSensorTransform(iDynTree::Transform& link_H_sensor) const
{
    if(this->pimpl->parent_link_index<0)
    {
        return false;
    }
    link_H_sensor = this->pimpl->link_H_sensor;
    return true;
    
} 

AngVelocity Gyroscope::predictMeasurement(const Twist& linkVel)
{
    
    if(this->pimpl->parent_link_index<0)
    {
        return (AngVelocity());
    }
    return(AngVelocity((this->pimpl->link_H_sensor * linkVel).getAngularVec3()));
}

/* to be implemented in the future after considering interface and requirements
 */
/*
 bool Gyroscope::getAngularVelocityOfLink( const iDynTree::AngVelocity& measured_angular_velocity,
                                                        iDynTree::AngVelocity& angular_velocity_of_link) const
    {
        return true;
    }*/


}
