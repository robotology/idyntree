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

#include "iDynTree/Sensors/Gyroscope.hpp"

//#include "kdl_codyco/undirectedtree.hpp"

#include "kdl_codyco/KDLConversions.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/Wrench.h"


namespace iDynTree {

struct Gyroscope::GyroscopePrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Index of the link to which the Gyroscope is connected
    int link;
    // Transform from the sensor
    Transform link_H_sensor;
    // Index of the parent junction
     int parent_junction_index;
    // Name of the parent junction
     std::string parent_junction_name;
    // Name of the link to which the Gyroscope is connected
    std::string linkName;

};


Gyroscope::Gyroscope()
{
    this->pimpl = new GyroscopePrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link = -1;
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

/*
bool SixAxisForceTorqueSensor::setAppliedWrenchLink(const int applied_wrench_index)
{
    this->pimpl->appliedWrenchLink = applied_wrench_index;
    return true;
}*/


bool Gyroscope::setLinkSensorTransform(const int link_index, const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link = link_index;
    this->pimpl->link_H_sensor = link_H_sensor;
    return true;
}

// bool SixAxisForceTorqueSensor::setSecondLinkSensorTransform(const int link_index, const iDynTree::Transform& link_H_sensor) const
// {
//     this->pimpl->link2 = link_index;
//     this->pimpl->link2_H_sensor = link_H_sensor;
//     return true;
// }

// bool Accelerometer::setParent(const std::string& parent)
// {
//     this->pimpl->parent_junction_name = parent;
//     return true;
// }
// 
// bool SixAxisForceTorqueSensor::setParentIndex(const int parent_index)
// {
//     this->pimpl->parent_junction_index = parent_index;
//     return true;
// }
// 
bool Gyroscope::isValid() const
{
    if( this->getName() == "" )
    {
        return false;
    }

    if( this->pimpl->link < 0 )
    {
        // Return false because the links is not appropriately setted
        return false;
    }

//     if( this->pimpl->link1 !=
//         this->pimpl->appliedWrenchLink &&
//         this->pimpl->link2 != this->pimpl->appliedWrenchLink)
//     {
//         return false;
//     }

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

/*
int Accelerometer::getAppliedWrenchLink() const
{
    return this->pimpl->appliedWrenchLink;
}*/

std::string Gyroscope::getParent() const
{
    return this->pimpl->parent_junction_name;
}

int Gyroscope::getParentIndex() const
{
    return this->pimpl->parent_junction_index;
}

bool Gyroscope::isLinkAttachedToSensor(const int link_index) const
{
    return (this->pimpl->link == link_index);
}


bool Gyroscope::getLinkSensorTransform(const int link_index, iDynTree::Transform& link_H_sensor) const
{
    if( this->pimpl->link == link_index )
    {
        link_H_sensor = this->pimpl->link_H_sensor;
        return true;
    }

//     if( this->pimpl->link2 == link_index )
//     {
//         link_H_sensor = this->pimpl->link2_H_sensor;
//         return true;
//     }

    return false;
}

    bool Gyroscope::getMeasurement()
    {
    }
    
    bool Gyroscope::getAngularVelocityOfLink(const int link_index,
                                                        const iDynTree::AngVelocity& measured_angular_velocity,
                                                        iDynTree::AngVelocity& angular_velocity_of_link) const
    {
        return true;
    }
//     assert(this->isValid());
// 
//     if( link_index == this->pimpl->link1 )
//     {
//         wrench_applied_on_link = this->pimpl->link1_H_sensor*measured_wrench;
//         // If the measure wrench is the one applied on the other link, change sign
//         if( this->getAppliedWrenchLink() != link_index )
//         {
//             wrench_applied_on_link = -wrench_applied_on_link;
//         }
// 
//         return true;
//     }
//     else if( link_index == this->pimpl->link2 )
//     {
//         wrench_applied_on_link = this->pimpl->link2_H_sensor*measured_wrench;
// 
//         if( this->getAppliedWrenchLink() != link_index )
//         {
//             wrench_applied_on_link = -wrench_applied_on_link;
//         }
// 
//         return true;
//     }
//     else
//     {
//         wrench_applied_on_link = iDynTree::Wrench();
//         return false;
//     }
// }

int Gyroscope::getLinkIndex() const
{
    return this->pimpl->link;
}

// int SixAxisForceTorqueSensor::getSecondLinkIndex() const
// {
//     return this->pimpl->link2;
// }
// 
bool Gyroscope::setLinkName(const std::string& name)
{
    this->pimpl->linkName = name;
    return true;
}
/*
bool SixAxisForceTorqueSensor::setSecondLinkName(const std::string& name)
{
    this->pimpl->link2Name = name;
    return true;
}

std::string SixAxisForceTorqueSensor::getFirstLinkName() const
{
    return this->pimpl->link1Name;
}

std::string SixAxisForceTorqueSensor::getSecondLinkName() const
{
    return this->pimpl->link2Name;
}*/

}
