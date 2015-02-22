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

#include "sensors.hpp"

#include "six_axis_ft_sensor.hpp"

#include <vector>
#include <map>

#include <kdl/frames.hpp>

#include <iostream>

namespace KDL {
namespace CoDyCo {


struct SensorsTree::SensorsTreePimpl
{
    std::vector< std::vector<Sensor *> > VecSensors;
    std::vector< std::map< std::string, unsigned int > > NamesSensors;
};

SensorsTree::SensorsTree():
    pimpl(new SensorsTreePimpl())
{
    //resize datastructures;
    this->pimpl->VecSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->NamesSensors.resize(NR_OF_SENSOR_TYPES);
}

void SensorsTree::constructor(const SensorsTree& other)
{
    this->pimpl = new SensorsTreePimpl();
    this->pimpl->VecSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->NamesSensors.resize(NR_OF_SENSOR_TYPES);
    for(int sens_type = 0; sens_type < NR_OF_SENSOR_TYPES; sens_type++ )
    {
        for(int sens = 0; sens < other.getNrOfSensors((SensorType)sens_type); sens++ )
        {
            this->pimpl->VecSensors[sens_type].push_back(other.pimpl->VecSensors[sens_type][sens]->clone());
            std::string sensor_name = other.getSensor((SensorType)sens_type,sens)->getName();
            this->pimpl->NamesSensors[sens_type].insert(std::pair<std::string,int>(sensor_name,sens));
        }
    }
}

SensorsTree::SensorsTree(const KDL::CoDyCo::SensorsTree& other)
{
    constructor(other);
}

SensorsTree& SensorsTree::operator=(const SensorsTree& other)
{
    if(this != &other)
    {
        destructor();
        constructor(other);
    }
    return *this;
}

void SensorsTree::destructor()
{
    for( int sensor_type = 0; sensor_type < NR_OF_SENSOR_TYPES; sensor_type++ )
    {
        for( int sensor_index = 0; sensor_index < this->pimpl->VecSensors[sensor_type].size(); sensor_index++ )
        {
            delete this->pimpl->VecSensors[sensor_type][sensor_index];
        }
    }
    this->pimpl->VecSensors.resize(0);
    this->pimpl->NamesSensors.resize(0);

    delete this->pimpl;
    this->pimpl = 0;
}


SensorsTree::~SensorsTree()
{
    this->destructor();
}

int SensorsTree::addSensor(const SensorType& sensor_type, const Sensor& sensor)
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        const Sensor * p_sensor = &sensor;
        const SixAxisForceTorqueSensor * p_ft_sensor = dynamic_cast<const SixAxisForceTorqueSensor *>(p_sensor);

        if( p_ft_sensor == 0 )
        {
            std::cerr << "[ERR] SensorsTree::addSensor error : sensor " << sensor.getName()
                      << " is not of the correct type" << std::endl;
            return -1;
        }

        if( !p_ft_sensor->isValid() )
        {
            std::cerr << "[ERR] SensorsTree::addSensor error : sensor  " << sensor.getName()
                      << "  isValid() method returns false" << std::endl;
            return -1;
        }

        //Allocate a copy of the sensor
        this->pimpl->VecSensors[SIX_AXIS_FORCE_TORQUE].push_back(new SixAxisForceTorqueSensor(*p_ft_sensor));
        int new_index = this->pimpl->VecSensors[SIX_AXIS_FORCE_TORQUE].size()-1;
        this->pimpl->NamesSensors[SIX_AXIS_FORCE_TORQUE].insert(std::pair<std::string,int>(sensor.getName(),new_index));

        assert( this->pimpl->NamesSensors[SIX_AXIS_FORCE_TORQUE].size() ==
                this->pimpl->VecSensors[SIX_AXIS_FORCE_TORQUE].size() );

        return new_index;
    }


    return -1;
}


unsigned int SensorsTree::getNrOfSensors(const SensorType & sensor_type) const
{
    return this->pimpl->VecSensors[sensor_type].size();
}

bool SensorsTree::getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, unsigned int & sensor_index)
{
    std::map< std::string, unsigned int >::const_iterator it;
    it = this->pimpl->NamesSensors[sensor_type].find(_sensor_name);
    if( it == this->pimpl->NamesSensors[sensor_type].end() )
    {
        return false;
    }
    else
    {
        sensor_index = it->second;
        return true;
    }
}

Sensor* SensorsTree::getSensor(const SensorType& sensor_type, int sensor_index) const
{
    if( sensor_index < getNrOfSensors(sensor_type) )
    {
        return this->pimpl->VecSensors[sensor_type][sensor_index];
    }
    else
    {
        return 0;
    }
}

///////////////////////////////////////////////////////////////////////////////
///// SensorTree
///////////////////////////////////////////////////////////////////////////////

struct SensorsMeasurements::SensorsMeasurementsPrivateAttributes
{
    std::vector< KDL::Wrench > SixAxisFTSensorsMeasurements;
};


SensorsMeasurements::SensorsMeasurements():
pimpl(new SensorsMeasurementsPrivateAttributes)
{
}

SensorsMeasurements::SensorsMeasurements(const SensorsMeasurements & other):
pimpl(new SensorsMeasurementsPrivateAttributes(*(other.pimpl)))
{
}

SensorsMeasurements& SensorsMeasurements::operator=(const SensorsMeasurements& other)
{
    if(this != &other)
    {
        *pimpl = *(other.pimpl);
    }
    return *this;
}


SensorsMeasurements::~SensorsMeasurements()
{
    delete this->pimpl;
}

bool SensorsMeasurements::setNrOfSensors(const SensorType& sensor_type, unsigned int nrOfSensors)
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        this->pimpl->SixAxisFTSensorsMeasurements.resize(nrOfSensors,KDL::Wrench::Zero());
        return true;
    }

    return false;
}

bool SensorsMeasurements::setMeasurement(const SensorType& sensor_type, const unsigned int& sensor_index, const Wrench& wrench)
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        this->pimpl->SixAxisFTSensorsMeasurements[sensor_index] = wrench;
        return true;
    }

    return false;
}

bool SensorsMeasurements::getMeasurement(const SensorType& sensor_type, const unsigned int& sensor_index, Wrench& wrench) const
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        wrench = this->pimpl->SixAxisFTSensorsMeasurements[sensor_index];
        return true;
    }

    return false;
}

unsigned int SensorsMeasurements::getNrOfSensors(const SensorType& sensor_type) const
{
    if( sensor_type < 0 ||
        sensor_type >= NR_OF_SENSOR_TYPES )
    {
        return this->pimpl->SixAxisFTSensorsMeasurements.size();
    }

    return 0;
}




}
}
