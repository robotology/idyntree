/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#include "iDynTree/Core/Wrench.h"
#include "iDynTree/Sensors/SixAxisForceTorqueSensor.h"

#include "iDynTree/Core/Transform.h"

#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/Link.h>

#include <iDynTree/Core/EigenHelpers.h>


#include <cassert>


namespace iDynTree {

struct SixAxisForceTorqueSensor::SixAxisForceTorqueSensorPrivateAttributes
{
    // Name/id of the sensor
    std::string name;
    // Index of the two links at which the SixAxisForceTorqueSensor is connected
    LinkIndex link1, link2, appliedWrenchLink;
    // Transform from the sensor
    Transform link1_H_sensor, link2_H_sensor;
    // Index of the parent junction
    JointIndex parent_junction_index;
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


bool SixAxisForceTorqueSensor::setAppliedWrenchLink(const LinkIndex applied_wrench_index)
{
    this->pimpl->appliedWrenchLink = applied_wrench_index;
    return true;
}


bool SixAxisForceTorqueSensor::setFirstLinkSensorTransform(const LinkIndex link_index, const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link1 = link_index;
    this->pimpl->link1_H_sensor = link_H_sensor;
    return true;
}

bool SixAxisForceTorqueSensor::setSecondLinkSensorTransform(const LinkIndex link_index, const iDynTree::Transform& link_H_sensor) const
{
    this->pimpl->link2 = link_index;
    this->pimpl->link2_H_sensor = link_H_sensor;
    return true;
}

bool SixAxisForceTorqueSensor::setParentJoint(const std::string& parent)
{
    this->pimpl->parent_junction_name = parent;
    return true;
}

bool SixAxisForceTorqueSensor::setParentJointIndex(const JointIndex &parent_index)
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

bool SixAxisForceTorqueSensor::updateIndices(const Model& model)
{
    if( !( (this->pimpl->appliedWrenchLink == this->pimpl->link1) ||
           (this->pimpl->appliedWrenchLink == this->pimpl->link2) ) )
    {
        return false;
    }

    std::string appliedWrenchLinkName;
    if( this->pimpl->appliedWrenchLink == this->pimpl->link1 )
    {
        appliedWrenchLinkName = this->pimpl->link1Name;
    }

    if( this->pimpl->appliedWrenchLink == this->pimpl->link2 )
    {
        appliedWrenchLinkName = this->pimpl->link2Name;
    }

    iDynTree::LinkIndex link1NewIndex = model.getLinkIndex(this->pimpl->link1Name);
    iDynTree::LinkIndex link2NewIndex = model.getLinkIndex(this->pimpl->link2Name);
    iDynTree::LinkIndex appliedWrenchLinkNewIndex = model.getLinkIndex(appliedWrenchLinkName);

    if( (link1NewIndex == iDynTree::LINK_INVALID_INDEX) ||
        (link2NewIndex == iDynTree::LINK_INVALID_INDEX) ||
        (appliedWrenchLinkNewIndex == iDynTree::LINK_INVALID_INDEX) )
    {
        return false;
    }

    this->pimpl->link1 = link1NewIndex;
    this->pimpl->link2 = link2NewIndex;
    this->pimpl->appliedWrenchLink = appliedWrenchLinkNewIndex;

    return true;
}

std::string SixAxisForceTorqueSensor::getName() const
{
    return this->pimpl->name;
}

SensorType SixAxisForceTorqueSensor::getSensorType() const
{
    return SIX_AXIS_FORCE_TORQUE;
}


LinkIndex SixAxisForceTorqueSensor::getAppliedWrenchLink() const
{
    return this->pimpl->appliedWrenchLink;
}

std::string SixAxisForceTorqueSensor::getParentJoint() const
{
    return this->pimpl->parent_junction_name;
}

JointIndex SixAxisForceTorqueSensor::getParentJointIndex() const
{
    return this->pimpl->parent_junction_index;
}

bool SixAxisForceTorqueSensor::isLinkAttachedToSensor(const LinkIndex link_index) const
{
    return (this->pimpl->link1 == link_index ||
            this->pimpl->link2 == link_index );
}


bool SixAxisForceTorqueSensor::getLinkSensorTransform(const LinkIndex link_index, iDynTree::Transform& link_H_sensor) const
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

bool SixAxisForceTorqueSensor::getWrenchAppliedOnLink(const LinkIndex link_index,
                                                      const Wrench& measured_wrench,
                                                      iDynTree::Wrench& wrench_applied_on_link) const
{
    assert(this->isValid());

    Wrench buffered_wrench;

    if( link_index == this->pimpl->link1 )
    {
        buffered_wrench = this->pimpl->link1_H_sensor*measured_wrench;
        // If the measure wrench is the one applied on the other link, change sign
        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link = -buffered_wrench;
        }
        else
        {
            wrench_applied_on_link = buffered_wrench;
        }

        return true;
    }
    else if( link_index == this->pimpl->link2 )
    {
        buffered_wrench = this->pimpl->link2_H_sensor*measured_wrench;

        if( this->getAppliedWrenchLink() != link_index )
        {
            wrench_applied_on_link = -buffered_wrench;
        }
        else
        {
            wrench_applied_on_link = buffered_wrench;
        }

        return true;
    }
    else
    {
        wrench_applied_on_link = iDynTree::Wrench::Zero();
        return false;
    }
}

bool SixAxisForceTorqueSensor::getWrenchAppliedOnLinkMatrix(const LinkIndex link_index,
                                                            Matrix6x6& wrench_applied_on_link_matrix) const
{
    assert(this->isValid());

    if( link_index == this->pimpl->link1 )
    {
        Transform & link_H_sensor = this->pimpl->link1_H_sensor;
        // If the measure wrench is the one applied on the other link, change sign
        if( this->getAppliedWrenchLink() != link_index )
        {
            toEigen(wrench_applied_on_link_matrix) = -toEigen(link_H_sensor.asAdjointTransformWrench());
        }
        else
        {
            toEigen(wrench_applied_on_link_matrix) = toEigen(link_H_sensor.asAdjointTransformWrench());
        }

        return true;
    }
    else if( link_index == this->pimpl->link2 )
    {
        Transform & link_H_sensor = this->pimpl->link2_H_sensor;

        if( this->getAppliedWrenchLink() != link_index )
        {
            toEigen(wrench_applied_on_link_matrix) = -toEigen(link_H_sensor.asAdjointTransformWrench());
        }
        else
        {
            toEigen(wrench_applied_on_link_matrix) = toEigen(link_H_sensor.asAdjointTransformWrench());
        }

        return true;
    }
    else
    {
        wrench_applied_on_link_matrix.zero();
        return false;
    }
}

bool SixAxisForceTorqueSensor::getWrenchAppliedOnLinkInverseMatrix(const LinkIndex link_index,
                                                                   Matrix6x6& wrench_applied_on_link_inverse_matrix) const
{
    assert(this->isValid());

    if( link_index == this->pimpl->link1 )
    {
        Transform sensor_H_link = this->pimpl->link1_H_sensor.inverse();
        // If the measure wrench is the one applied on the other link, change sign
        if( this->getAppliedWrenchLink() != link_index )
        {
            toEigen(wrench_applied_on_link_inverse_matrix) = -toEigen(sensor_H_link.asAdjointTransformWrench());
        }
        else
        {
            toEigen(wrench_applied_on_link_inverse_matrix) = toEigen(sensor_H_link.asAdjointTransformWrench());
        }

        return true;
    }
    else if( link_index == this->pimpl->link2 )
    {
        Transform sensor_H_link = this->pimpl->link2_H_sensor.inverse();

        if( this->getAppliedWrenchLink() != link_index )
        {
            toEigen(wrench_applied_on_link_inverse_matrix) = -toEigen(sensor_H_link.asAdjointTransformWrench());
        }
        else
        {
            toEigen(wrench_applied_on_link_inverse_matrix) = toEigen(sensor_H_link.asAdjointTransformWrench());
        }

        return true;
    }
    else
    {
        assert(false);
        wrench_applied_on_link_inverse_matrix.zero();
        return false;
    }
}


LinkIndex SixAxisForceTorqueSensor::getFirstLinkIndex() const
{
    return this->pimpl->link1;
}

LinkIndex SixAxisForceTorqueSensor::getSecondLinkIndex() const
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

Wrench SixAxisForceTorqueSensor::predictMeasurement(const Traversal& traversal, const LinkInternalWrenches& intWrenches)
{
    Wrench simulated_measurement;

    //Check that the input size is consistent
    assert(this->isValid());
    assert(this->getFirstLinkIndex() >= 0 && this->getFirstLinkIndex() < static_cast<LinkIndex>(traversal.getNrOfVisitedLinks()));
    assert(this->getSecondLinkIndex() >= 0 && this->getSecondLinkIndex() < static_cast<LinkIndex>(traversal.getNrOfVisitedLinks()));

    // The intWrenches vector is assumed to be the output of the RNEADynamicPhase function called
    // with the passed traversal.
    // ie intWrench[i] is the force applied by link i on the link traversal.getParent(i),
    // expressed in the refernce frame of link i
    // From this information, we can "simulate" the output that we could expect on this sensor

    // First we get the two links attached to this ft sensor, and we check which one is the
    // parent and which one is the child in the dynamic_traversal Traversal
    LinkIndex child_link = LINK_INVALID_INDEX;
    LinkIndex parent_link = LINK_INVALID_INDEX;
    if( traversal.getParentLinkFromLinkIndex(this->getFirstLinkIndex()) != 0 &&
        traversal.getParentLinkFromLinkIndex(this->getFirstLinkIndex())->getIndex() == this->getSecondLinkIndex() )
    {
        child_link = this->getFirstLinkIndex();
        parent_link = this->getSecondLinkIndex();
    }
    else
    {
        assert( traversal.getParentLinkFromLinkIndex(this->getSecondLinkIndex())->getIndex() == this->getFirstLinkIndex());
        child_link = this->getSecondLinkIndex();
        parent_link = this->getFirstLinkIndex();
    }

    // if the child_link is the link to which the measured wrench is applied, the sign between the
    // measured_wrench and f[child_link] is consistent, otherwise we have to change the sign


    // To simulate the sensor, we have to translate f[child] in the sensor frame
    // with the appriopriate sign
    iDynTree::Transform child_link_H_sensor;
    this->getLinkSensorTransform(child_link,child_link_H_sensor);
    if( this->getAppliedWrenchLink() == parent_link  )
    {
        simulated_measurement = -(child_link_H_sensor.inverse()*intWrenches(child_link));
    }
    else
    {
        simulated_measurement = (child_link_H_sensor.inverse()*intWrenches(child_link));
        assert( this->getAppliedWrenchLink() == child_link );
    }

    return simulated_measurement;
}

std::string SixAxisForceTorqueSensor::toString(const Model&  /*model*/) const
{
    std::stringstream ss;

    ss << "Sensor " << this->getName() << std::endl;
    ss << " is attached to joint " << this->getParentJoint() << " ( " << this->getParentJointIndex() << " ) " << std::endl;
    ss << " that connects  " << this->getFirstLinkName() << " ( " << this->getFirstLinkIndex() << " ) and "
                             << this->getSecondLinkName() << " ( " << this->getSecondLinkIndex() << std::endl;

    return ss.str();
}


}
