/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Sensors/GyroscopeSensor.h"
#include "iDynTree/Core/Transform.h"
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Core/Twist.h"


namespace iDynTree {

struct GyroscopeSensor::GyroscopePrivateAttributes
{
    // Name/id of the sensor
    std::string name;
   // Transform from the link to the sensor
    Transform link_H_sensor;
    // Index of the parent link
    LinkIndex parent_link_index;
    // Name of the parent link
     std::string parent_link_name;
};


GyroscopeSensor::GyroscopeSensor()
{
    this->pimpl = new GyroscopePrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";
}

GyroscopeSensor::GyroscopeSensor(const GyroscopeSensor& other):
    pimpl(new GyroscopePrivateAttributes(*(other.pimpl)))
{

}

GyroscopeSensor& GyroscopeSensor::operator=(const GyroscopeSensor& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


GyroscopeSensor::~GyroscopeSensor()
{
    delete this->pimpl;
}

bool GyroscopeSensor::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool GyroscopeSensor::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor)
{
    this->pimpl->link_H_sensor = link_H_sensor;
    return true;
}

bool GyroscopeSensor::setParentLink(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool GyroscopeSensor::setParentLinkIndex(const LinkIndex &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;
}

bool GyroscopeSensor::isValid() const
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

Sensor* GyroscopeSensor::clone() const
{
    return (Sensor *)new GyroscopeSensor(*this);
}

bool GyroscopeSensor::updateIndices(const Model& model)
{
    iDynTree::LinkIndex linkNewIndex = model.getLinkIndex(this->pimpl->parent_link_name);

    if( linkNewIndex == iDynTree::LINK_INVALID_INDEX )
    {
        return false;
    }

    this->pimpl->parent_link_index = linkNewIndex;

    return true;
}

std::string GyroscopeSensor::getName() const
{
    return this->pimpl->name;
}

SensorType GyroscopeSensor::getSensorType() const
{
    return GYROSCOPE;
}


std::string GyroscopeSensor::getParentLink() const
{
    return this->pimpl->parent_link_name;
}

LinkIndex GyroscopeSensor::getParentLinkIndex() const
{
    return this->pimpl->parent_link_index;
}


Transform GyroscopeSensor::getLinkSensorTransform() const
{
    return(this->pimpl->link_H_sensor);

}

AngVelocity GyroscopeSensor::predictMeasurement(const Twist& linkVel)
{
    AngVelocity angVel(0,0,0);
    if(this->pimpl->parent_link_index>=0)
    {
        angVel = ((this->pimpl->link_H_sensor.inverse() * linkVel).getAngularVec3());
    }
    return(angVel);
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
