/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h"

#include "iDynTree/Core/Transform.h"

#include "iDynTree/Core/SpatialAcc.h"
#include "iDynTree/Core/Twist.h"

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree {

struct ThreeAxisForceTorqueContactSensor::ThreeAxisForceTorqueContactSensorPrivateAttributes
{
    std::string name;
    Transform link_H_sensor;
    LinkIndex parent_link_index;
    std::string parent_link_name;

    std::vector<Position> m_loadCellLocations;
};


ThreeAxisForceTorqueContactSensor::ThreeAxisForceTorqueContactSensor()
{
    this->pimpl = new ThreeAxisForceTorqueContactSensorPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link_H_sensor = Transform::Identity();
    this->pimpl->parent_link_index = -1;
    this->pimpl->parent_link_name = "";

}

ThreeAxisForceTorqueContactSensor::ThreeAxisForceTorqueContactSensor(const ThreeAxisForceTorqueContactSensor& other):
    pimpl(new ThreeAxisForceTorqueContactSensorPrivateAttributes(*(other.pimpl)))
{

}

ThreeAxisForceTorqueContactSensor& ThreeAxisForceTorqueContactSensor::operator=(const ThreeAxisForceTorqueContactSensor& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


ThreeAxisForceTorqueContactSensor::~ThreeAxisForceTorqueContactSensor()
{
    delete this->pimpl;
}

bool ThreeAxisForceTorqueContactSensor::setName(const std::string& _name)
{
    this->pimpl->name = _name;
    return true;
}


bool ThreeAxisForceTorqueContactSensor::setLinkSensorTransform(const iDynTree::Transform& link_H_sensor)
{
      this->pimpl->link_H_sensor = link_H_sensor;
      return true;
}


bool ThreeAxisForceTorqueContactSensor::setParentLink(const std::string& parent)
{
    this->pimpl->parent_link_name = parent;
    return true;
}

bool ThreeAxisForceTorqueContactSensor::setParentLinkIndex(const LinkIndex &parent_index)
{
    this->pimpl->parent_link_index = parent_index;
    return true;

}

std::string ThreeAxisForceTorqueContactSensor::getParentLink() const
{
    return(this->pimpl->parent_link_name);
}

LinkIndex ThreeAxisForceTorqueContactSensor::getParentLinkIndex() const
{
    return(this->pimpl->parent_link_index);
}

bool ThreeAxisForceTorqueContactSensor::isValid() const
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

Sensor* ThreeAxisForceTorqueContactSensor::clone() const
{
    return (Sensor *)new ThreeAxisForceTorqueContactSensor(*this);
}

bool ThreeAxisForceTorqueContactSensor::updateIndices(const Model& model)
{
    iDynTree::LinkIndex linkNewIndex = model.getLinkIndex(this->pimpl->parent_link_name);

    if( linkNewIndex == iDynTree::LINK_INVALID_INDEX )
    {
        return false;
    }

    this->pimpl->parent_link_index = linkNewIndex;

    return true;
}

std::string ThreeAxisForceTorqueContactSensor::getName() const
{
    return this->pimpl->name;
}

SensorType ThreeAxisForceTorqueContactSensor::getSensorType() const
{
    return THREE_AXIS_FORCE_TORQUE_CONTACT;
}

Transform ThreeAxisForceTorqueContactSensor::getLinkSensorTransform() const
{
    return(this->pimpl->link_H_sensor);
}

void ThreeAxisForceTorqueContactSensor::setLoadCellLocations(std::vector<Position> &loadCellLocations)
{
    pimpl->m_loadCellLocations = loadCellLocations;
}

std::vector<Position>  ThreeAxisForceTorqueContactSensor::getLoadCellLocations() const
{
    return pimpl->m_loadCellLocations;
}

Vector3  ThreeAxisForceTorqueContactSensor::computeThreeAxisForceTorqueFromLoadCellMeasurements(VectorDynSize& loadCellMeasurements) const
{
    if (loadCellMeasurements.size() != pimpl->m_loadCellLocations.size())
    {
        reportError("ThreeAxisForceTorqueContactSensor","computeThreeAxisForceTorqueFromLoadCellMeasurements", "loadCellMeasurements has the wrong size");
        Vector3 zero; zero.zero();
        return zero;
    }

    Wrench totalWrench;
    totalWrench.zero();

    for (size_t i=0; i < loadCellMeasurements.size(); i++)
    {
        // We assume that the total forcetorque is given by the sum of pure forces on the z axis measured by the load cells
        Wrench loadCellWrench = Wrench(LinearForceVector3(0.0, 0.0, loadCellMeasurements(i)*1.0), AngularForceVector3(0.0, 0.0, 0.0));
        Transform sensor_H_loadCell(Rotation::Identity(), pimpl->m_loadCellLocations[i]);
        totalWrench = totalWrench + sensor_H_loadCell*loadCellWrench;
    }

    // Project the force torque
    Vector3 ret;
    ret(0) = totalWrench.getLinearVec3()(2);
    ret(1) = totalWrench.getAngularVec3()(0);
    ret(2) = totalWrench.getAngularVec3()(1);

    return ret;
}

Position ThreeAxisForceTorqueContactSensor::computeCenterOfPressureFromLoadCellMeasurements(VectorDynSize& loadCellMeasurements) const
{
    if (loadCellMeasurements.size() != pimpl->m_loadCellLocations.size())
    {
        reportError("ThreeAxisForceTorqueContactSensor","computeThreeAxisForceTorqueFromLoadCellMeasurements", "loadCellMeasurements has the wrong size");
        Position zeroVec; zeroVec.zero();
        return zeroVec;
    }

    // We assume that the location of the load cells is on the XY plane. In this case, the cop is just the average of the locations weighted on the measurements
    double sumOfMeasurementsInN=0.0;
    Vector3 weightedSumOfLocations; weightedSumOfLocations.zero();
    for (size_t i=0; i < loadCellMeasurements.size(); i++)
    {
        sumOfMeasurementsInN += loadCellMeasurements(i);
        toEigen(weightedSumOfLocations) += loadCellMeasurements(i)*toEigen(pimpl->m_loadCellLocations[i]);
    }

    if (sumOfMeasurementsInN < 1e-9)
    {
        reportError("ThreeAxisForceTorqueContactSensor","computeThreeAxisForceTorqueFromLoadCellMeasurements", "loadCellMeasurements sum is less than 1e-9");
        Position zeroVec; zeroVec.zero();
        return zeroVec;
    }

    Position ret;
    toEigen(ret) = toEigen(weightedSumOfLocations)/sumOfMeasurementsInN;
    return ret;
}


}
