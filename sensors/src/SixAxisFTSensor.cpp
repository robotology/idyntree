/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
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
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Sensors/SixAxisFTSensor.hpp"

#include "iDynTree/Core/Transform.h"


#include <cassert>


namespace iDynTree {

struct SixAxisForceTorqueSensor::SixAxisForceTorqueSensorPrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Index of the two links at which the SixAxisForceTorqueSensor is connected
    int link1, link2, appliedWrenchLink;
    // Transform from the sensor
    Transform link1_H_sensor, link2_H_sensor;
    // Index of the parent junction
    int parent_junction_index;
    // Name of the parent junction
    std::string parent_junction_name;
    // Name of the two links at which the SixAxisForceTorqueSensor is connected
    std::string link1Name, link2Name;

};


SixAxisForceTorqueSensor::SixAxisForceTorqueSensor()
{
    this->pimpl = new SixAxisForceTorqueSensorPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link1 = this->pimpl->link2 = this->pimpl->appliedWrenchLink = -1;
}

SixAxisForceTorqueSensor::SixAxisForceTorqueSensor(const SixAxisForceTorqueSensor& other):
    pimpl(new SixAxisForceTorqueSensorPrivateAttributes(*(other.pimpl)))
{

}

SixAxisForceTorqueSensor& SixAxisForceTorqueSensor::operator=(const SixAxisForceTorqueSensor& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


SixAxisForceTorqueSensor::~SixAxisForceTorqueSensor()
{
    delete this->pimpl;
}

bool SixAxisForceTorqueSensor::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool SixAxisForceTorqueSensor::setAppliedWrenchLink(const int applied_wrench_index)
{
    this->pimpl->appliedWrenchLink = applied_wrench_index;
    return true;
}


bool SixAxisForceTorqueSensor::setFirstLinkSensorTransform(const int link_index, const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link1 = link_index;
    this->pimpl->link1_H_sensor = link_H_sensor;
    return true;
}

bool SixAxisForceTorqueSensor::setSecondLinkSensorTransform(const int link_index, const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link2 = link_index;
    this->pimpl->link2_H_sensor = link_H_sensor;
    return true;
}

bool SixAxisForceTorqueSensor::setParent(const std::string& parent)
{
    this->pimpl->parent_junction_name = parent;
    return true;
}

bool SixAxisForceTorqueSensor::setParentIndex(const int &parent_index)
{
    this->pimpl->parent_junction_index = parent_index;
    return true;
}

bool SixAxisForceTorqueSensor::isValid() const
{
    if( this->getName() == "" )
    {
        return false;
    }

    if( this->pimpl->link1 < 0 &&
        this->pimpl->link2 < 0 &&
        this->pimpl->appliedWrenchLink < 0 )
    {
        // Return false because the links is not appropriately setted
        return false;
    }

    if( this->pimpl->link1 !=
        this->pimpl->appliedWrenchLink &&
        this->pimpl->link2 != this->pimpl->appliedWrenchLink)
    {
        return false;
    }

    return true;
}

Sensor* SixAxisForceTorqueSensor::clone() const
{
    return (Sensor *)new SixAxisForceTorqueSensor(*this);
}


std::string SixAxisForceTorqueSensor::getName() const
{
    return this->pimpl->name;
}

SensorType SixAxisForceTorqueSensor::getSensorType() const
{
    return SIX_AXIS_FORCE_TORQUE;
}


int SixAxisForceTorqueSensor::getAppliedWrenchLink() const
{
    return this->pimpl->appliedWrenchLink;
}

std::string SixAxisForceTorqueSensor::getParent() const
{
    return this->pimpl->parent_junction_name;
}

int SixAxisForceTorqueSensor::getParentIndex() const
{
    return this->pimpl->parent_junction_index;
}

bool SixAxisForceTorqueSensor::isLinkAttachedToSensor(const int link_index) const
{
    return (this->pimpl->link1 == link_index ||
            this->pimpl->link2 == link_index );
}


bool SixAxisForceTorqueSensor::getLinkSensorTransform(const int link_index, iDynTree::Transform& link_H_sensor) const
{
    if( this->pimpl->link1 == link_index )
    {
        link_H_sensor = this->pimpl->link1_H_sensor;
        return true;
    }

    if( this->pimpl->link2 == link_index )
    {
        link_H_sensor = this->pimpl->link2_H_sensor;
        return true;
    }

    return false;
}

bool SixAxisForceTorqueSensor::getWrenchAppliedOnLink(const int link_index,
                                                      const Wrench& measured_wrench,
                                                      iDynTree::Wrench& wrench_applied_on_link) const
{
    assert(this->isValid());

    if( link_index == this->pimpl->link1 )
    {
        wrench_applied_on_link = this->pimpl->link1_H_sensor*measured_wrench;
        // If the measure wrench is the one applied on the other link, change sign
        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link = -wrench_applied_on_link;
        }

        return true;
    }
    else if( link_index == this->pimpl->link2 )
    {
        wrench_applied_on_link = this->pimpl->link2_H_sensor*measured_wrench;

        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link = -wrench_applied_on_link;
        }

        return true;
    }
    else
    {
        wrench_applied_on_link = iDynTree::Wrench();
        return false;
    }
}

int SixAxisForceTorqueSensor::getFirstLinkIndex() const
{
    return this->pimpl->link1;
}

int SixAxisForceTorqueSensor::getSecondLinkIndex() const
{
    return this->pimpl->link2;
}

bool SixAxisForceTorqueSensor::setFirstLinkName(const std::string& name)
{
    this->pimpl->link1Name = name;
    return true;
}

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
}

}
