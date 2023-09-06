// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include "iDynTree/AccelerometerSensor.h"

#include "iDynTree/Transform.h"

#include "iDynTree/SpatialAcc.h"
#include "iDynTree/Twist.h"

#include "iDynTree/Model.h"


namespace iDynTree {

struct AccelerometerSensor::AccelerometerPrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Transform from the link to the sensor
    Transform link_H_sensor;
    // Index of the parent link
    LinkIndex parent_link_index;
    // Name of the parent link
     std::string parent_link_name;
    // Name of the link to which the Accelerometer is connected
};


AccelerometerSensor::AccelerometerSensor()
{
    this->pimpl = new AccelerometerPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";

}

AccelerometerSensor::AccelerometerSensor(const AccelerometerSensor& other):
    pimpl(new AccelerometerPrivateAttributes(*(other.pimpl)))
{

}

AccelerometerSensor& AccelerometerSensor::operator=(const AccelerometerSensor& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


AccelerometerSensor::~AccelerometerSensor()
{
    delete this->pimpl;
}

bool AccelerometerSensor::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool AccelerometerSensor::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor)
{
      this->pimpl->link_H_sensor = link_H_sensor;
      return true;
}


bool AccelerometerSensor::setParentLink(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool AccelerometerSensor::setParentLinkIndex(const LinkIndex &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;

}

std::string AccelerometerSensor::getParentLink() const
{
    return(this->pimpl->parent_link_name);
}

LinkIndex AccelerometerSensor::getParentLinkIndex() const
{
    return(this->pimpl->parent_link_index);
}

bool AccelerometerSensor::isValid() const
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

Sensor* AccelerometerSensor::clone() const
{
    return (Sensor *)new AccelerometerSensor(*this);
}

bool AccelerometerSensor::updateIndices(const Model& model)
{
    iDynTree::LinkIndex linkNewIndex = model.getLinkIndex(this->pimpl->parent_link_name);

    if( linkNewIndex == iDynTree::LINK_INVALID_INDEX )
    {
        return false;
    }

    this->pimpl->parent_link_index = linkNewIndex;

    return true;
}

std::string AccelerometerSensor::getName() const
{
    return this->pimpl->name;
}

SensorType AccelerometerSensor::getSensorType() const
{
    return ACCELEROMETER;
}

Transform AccelerometerSensor::getLinkSensorTransform() const
{
    return(this->pimpl->link_H_sensor);
}

LinAcceleration AccelerometerSensor::predictMeasurement(const SpatialAcc& linkAcc, const iDynTree::Twist& linkTwist)
{
    LinAcceleration returnAcc(0,0,0);
    if( this->pimpl->parent_link_index >= 0)
    {
        iDynTree::Twist localVelocity = this->pimpl->link_H_sensor.inverse() * linkTwist;
        returnAcc = ((this->pimpl->link_H_sensor.inverse() * linkAcc).getLinearVec3() + (localVelocity.getAngularVec3()).cross(localVelocity.getLinearVec3()));
    }

    return(returnAcc);
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
