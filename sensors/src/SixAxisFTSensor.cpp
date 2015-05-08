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

#include "SixAxisFTSensor.hpp"

#include "kdl_codyco/undirectedtree.hpp"

#include <kdl/frames.hpp>



namespace KDL {
namespace CoDyCo {

struct SixAxisForceTorqueSensor::SixAxisForceTorqueSensorPrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Index of the two links at which the SixAxisForceTorqueSensor is connected
    int link1, link2, appliedWrenchLink;
    // Transform from the sensor
    KDL::Frame link1_H_sensor, link2_H_sensor;
    // Name of the parent junction
    std::string parent_junction_name;
    // Index of the parent junction
    int parent_junction_index;
};


SixAxisForceTorqueSensor::SixAxisForceTorqueSensor()
{
    this->pimpl = new SixAxisForceTorqueSensorPrivateAttributes;

    this->pimpl->name = "";
    this->pimpl->link1 = this->pimpl->link2 = this->pimpl->appliedWrenchLink = -1;
}

SixAxisForceTorqueSensor::SixAxisForceTorqueSensor(const KDL::CoDyCo::SixAxisForceTorqueSensor& other):
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


bool SixAxisForceTorqueSensor::setFirstLinkSensorTransform(const int link_index, const KDL::Frame& link_H_sensor) const
{
    this->pimpl->link1 = link_index;
    this->pimpl->link1_H_sensor = link_H_sensor;
    return true;
}

bool SixAxisForceTorqueSensor::setSecondLinkSensorTransform(const int link_index, const KDL::Frame& link_H_sensor) const
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

bool SixAxisForceTorqueSensor::setParentIndex(const int parent_index)
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


bool SixAxisForceTorqueSensor::getLinkSensorTransform(const int link_index, KDL::Frame& link_H_sensor) const
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
                                                      Wrench& wrench_applied_on_link) const
{
    assert(this->isValid());

    if( link_index == this->pimpl->link1 )
    {
        wrench_applied_on_link = this->pimpl->link1_H_sensor*measured_wrench;
        // If the measure wrench is the one applied on the other link, change sign
        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link.ReverseSign();
        }

        return true;
    }
    else if( link_index == this->pimpl->link2 )
    {
        wrench_applied_on_link = this->pimpl->link2_H_sensor*measured_wrench;

        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link.ReverseSign();
        }

        return true;
    }
    else
    {
        wrench_applied_on_link = KDL::Wrench::Zero();
        return false;
    }
}

bool SixAxisForceTorqueSensor::simulateMeasurement(Traversal& dynamic_traversal,
                                                   std::vector< Wrench > f,
                                                   Wrench& simulated_measurement)
{
    //Check that the input size is consistent
    assert(f.size() == dynamic_traversal.getNrOfVisitedLinks());
    assert(this->isValid());
    assert(this->pimpl->link1 > 0 && this->pimpl->link1 < dynamic_traversal.getNrOfVisitedLinks());
    assert(this->pimpl->link2 > 0 && this->pimpl->link2 < dynamic_traversal.getNrOfVisitedLinks());


    // The f vector is assume to be the output of the rneaDynamicLoop function,
    // ie f[i] is the force applied by link i on the link dynamic_traversal.getParent(i),
    // expressed in the refernce frame of link i
    // From this information, we can "simulate" the output that we could expect on this sensor

    // First we get the two links attached to this ft sensor, and we check which one is the
    // parent and which one is the child in the dynamic_traversal Traversal
    int child_link = -1;
    int parent_link = -1;
    if( dynamic_traversal.getParentLink(this->pimpl->link1)->getLinkIndex() == this->pimpl->link2 )
    {
        child_link = this->pimpl->link1;
        parent_link = this->pimpl->link2;
    }
    else
    {
        assert(dynamic_traversal.getParentLink(this->pimpl->link2)->getLinkIndex() == this->pimpl->link1 );
        child_link = this->pimpl->link2;
        parent_link = this->pimpl->link1;
    }

    // if the child_link is the link to which the measured wrench is applied, the sign between the
    // measured_wrench and f[child_link] is consistent, otherwise we have to change the sign

    double sign = 0.0;
    if( this->getAppliedWrenchLink() == parent_link )
    {
        sign = -1.0;
    }
    else
    {
        assert( this->getAppliedWrenchLink() == child_link );
        sign = 1.0;
    }

    // To simulate the sensor, we have to translate f[child] in the sensor frame
    // with the appriopriate sign
    KDL::Frame child_link_H_sensor;
    this->getLinkSensorTransform(child_link,child_link_H_sensor);
    simulated_measurement = sign*(child_link_H_sensor.Inverse()*f[child_link]);

    return true;
}







}
}
