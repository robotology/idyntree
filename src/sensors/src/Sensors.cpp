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



#include <vector>
#include <map>

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/LinearMotionVector3.h>

// #include "iDynTree/Sensors/IMeasurement.hpp"
#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Sensors/SixAxisFTSensor.h>
#include <iDynTree/Sensors/Accelerometer.h>
#include <iDynTree/Sensors/Gyroscope.h>




#include <iostream>

namespace iDynTree {

Sensor::~Sensor()
{

}


///////////////////////////////////////////////////////////////////////////////
///// SensorList
///////////////////////////////////////////////////////////////////////////////

struct SensorsList::SensorsListPimpl
{
    std::vector< std::vector<Sensor *> > VecSensors;
    std::vector< std::map< std::string, unsigned int > > NamesSensors;
};

SensorsList::SensorsList():
    pimpl(new SensorsListPimpl())
{
    //resize datastructures;
    this->pimpl->VecSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->NamesSensors.resize(NR_OF_SENSOR_TYPES);
}

void SensorsList::constructor(const SensorsList& other)
{
    this->pimpl = new SensorsListPimpl();
    this->pimpl->VecSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->NamesSensors.resize(NR_OF_SENSOR_TYPES);
    for(int sens_type = 0; sens_type < NR_OF_SENSOR_TYPES; sens_type++ )
    {
        for(unsigned int sens = 0; sens < other.getNrOfSensors((SensorType)sens_type); sens++ )
        {
            this->pimpl->VecSensors[sens_type].push_back(other.pimpl->VecSensors[sens_type][sens]->clone());
            std::string sensor_name = other.getSensor((SensorType)sens_type,sens)->getName();
            this->pimpl->NamesSensors[sens_type].insert(std::pair<std::string,int>(sensor_name,sens));
        }
    }
}

SensorsList::SensorsList(const iDynTree::SensorsList& other)
{
    constructor(other);
}

SensorsList& SensorsList::operator=(const SensorsList& other)
{
    if(this != &other)
    {
        destructor();
        constructor(other);
    }
    return *this;
}

void SensorsList::destructor()
{
    for( int sensor_type = 0; sensor_type < NR_OF_SENSOR_TYPES; sensor_type++ )
    {
        for(unsigned int sensor_index = 0;
            sensor_index < this->pimpl->VecSensors[sensor_type].size(); sensor_index++ )
        {
            delete this->pimpl->VecSensors[sensor_type][sensor_index];
        }
    }
    this->pimpl->VecSensors.resize(0);
    this->pimpl->NamesSensors.resize(0);

    delete this->pimpl;
    this->pimpl = 0;
}


SensorsList::~SensorsList()
{
    this->destructor();
}

int SensorsList::addSensor(const Sensor& sensor)
 {
    Sensor *newSensor = sensor.clone();
    if( ! newSensor->isValid() )
    {
         std::cerr << "[ERR] SensorsTree::addSensor error : sensor  " << sensor.getName()
                       << "  isValid() method returns false" << std::endl;
         delete newSensor;
             return -1;
    }

    if( !(newSensor->getSensorType() >= 0 && newSensor->getSensorType() < NR_OF_SENSOR_TYPES) )
    {
         std::cerr << "[ERR] SensorsTree::addSensor error : sensor  " << sensor.getName()
                       << " has an unknown sensor type " << newSensor->getSensorType() << std::endl;
         delete newSensor;
             return -1;
    }

    this->pimpl->VecSensors[newSensor->getSensorType()].push_back(newSensor);
    int new_index = this->pimpl->VecSensors[newSensor->getSensorType()].size()-1;
    this->pimpl->NamesSensors[newSensor->getSensorType()].insert(std::pair<std::string,int>(newSensor->getName(),new_index));

    return new_index;
}

unsigned int SensorsList::getNrOfSensors(const SensorType & sensor_type) const
{
    return this->pimpl->VecSensors[sensor_type].size();
}

bool SensorsList::getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, unsigned int & sensor_index) const
{
    std::map< std::string, unsigned int >::const_iterator it;
    it = this->pimpl->NamesSensors[sensor_type].find(_sensor_name);
    if( it == this->pimpl->NamesSensors[sensor_type].end() )
    {
        std::cerr << "[ERROR] getSensorIndex did not find sensor " << _sensor_name << std::endl;
        return false;
    }
    else
    {
        sensor_index = it->second;
        return true;
    }
}

int SensorsList::getSensorIndex(const SensorType& sensor_type, const std::string& _sensor_name) const
{
    unsigned int retUnsignedVal;
    int retVal;
    bool ok = getSensorIndex(sensor_type,_sensor_name,retUnsignedVal);

    if( !ok )
    {
        retVal = -1;
    }
    else
    {
        // \todo TODO check overflow
        retVal = (int) retUnsignedVal;
    }

    return retVal;
}


Sensor* SensorsList::getSensor(const SensorType& sensor_type, int sensor_index) const
{
    if( sensor_index < (int)getNrOfSensors(sensor_type) && sensor_index >= 0 )
    {
        return this->pimpl->VecSensors[sensor_type][sensor_index];
    }
    else
    {
        std::cerr << "[ERROR] getSensor did not find sensor "
                  << sensor_index << " of type " << sensor_type
                  << std::endl;
        return 0;
    }
}



///////////////////////////////////////////////////////////////////////////////
///// SensorMeasurements
///////////////////////////////////////////////////////////////////////////////

struct SensorsMeasurements::SensorsMeasurementsPrivateAttributes
{
    std::vector< iDynTree::Wrench > SixAxisFTSensorsMeasurements;
    std::vector< iDynTree::AngVelocity > GyroscopeMeasurements;
    std::vector< iDynTree::LinAcceleration> AccelerometerMeasurements;
};


 SensorsMeasurements::SensorsMeasurements() : pimpl(new SensorsMeasurementsPrivateAttributes)
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
//     if( sensor_type == SIX_AXIS_FORCE_TORQUE )
//     {
//         this->pimpl->SixAxisFTSensorsMeasurements.resize(nrOfSensors);
//         return true;
//     }
    
        unsigned int returnVal = 0;
    switch (sensor_type)
    {
        case SIX_AXIS_FORCE_TORQUE :
            this->pimpl->SixAxisFTSensorsMeasurements.resize(nrOfSensors);
            break;
        case ACCELEROMETER : 
            this->pimpl->AccelerometerMeasurements.resize(nrOfSensors);
            break;
        case GYROSCOPE : 
            this->pimpl->GyroscopeMeasurements.resize(nrOfSensors);
        default :
            returnVal = 0;
    }
   return(returnVal);


    return true;
}

bool SensorsMeasurements::setMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         const iDynTree::Wrench &measurement )
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        if( sensor_index < this->pimpl->SixAxisFTSensorsMeasurements.size() )
        {
            this->pimpl->SixAxisFTSensorsMeasurements[sensor_index] = measurement;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] setMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->SixAxisFTSensorsMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

bool SensorsMeasurements::setMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         const iDynTree::LinAcceleration & measurement)
{
    if( sensor_type == ACCELEROMETER )
    {
        if( sensor_index < this->pimpl->AccelerometerMeasurements.size() )
        {
            this->pimpl->AccelerometerMeasurements[sensor_index] = measurement;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] setMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->AccelerometerMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

bool SensorsMeasurements::setMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         const iDynTree::AngVelocity & measurement)
{
    if( sensor_type == GYROSCOPE )
    {
        if( sensor_index < this->pimpl->GyroscopeMeasurements.size() )
        {
            this->pimpl->GyroscopeMeasurements[sensor_index] = measurement;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] setMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->GyroscopeMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}
bool SensorsMeasurements::getMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         Wrench& measurement) const
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        if( sensor_index < this->pimpl->SixAxisFTSensorsMeasurements.size() )
        {
            measurement = this->pimpl->SixAxisFTSensorsMeasurements[sensor_index];
            return true;
        }
        else
        {
            std::cerr << "[ERROR] getMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this-> pimpl->SixAxisFTSensorsMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

bool SensorsMeasurements::getMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         iDynTree::LinAcceleration & measurement) const
{
    if( sensor_type == ACCELEROMETER )
    {
        if( sensor_index < this->pimpl->AccelerometerMeasurements.size() )
        {
            measurement = this->pimpl->AccelerometerMeasurements[sensor_index];
            return true;
        }
        else
        {
            std::cerr << "[ERROR] getMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->AccelerometerMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}
bool SensorsMeasurements::getMeasurement(const SensorType& sensor_type, 
                                         const unsigned int& sensor_index, 
                                         iDynTree::AngVelocity& measurement) const
{
    if( sensor_type == GYROSCOPE )
    {
        if( sensor_index < this->pimpl->GyroscopeMeasurements.size() )
        {
            measurement = this->pimpl->GyroscopeMeasurements[sensor_index];
            return true;
        }
        else
        {
            std::cerr << "[ERROR] getMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->GyroscopeMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

unsigned int SensorsMeasurements::getNrOfSensors(const SensorType& sensor_type) const
{
    unsigned int returnVal = 0;
    switch (sensor_type)
    {
        case SIX_AXIS_FORCE_TORQUE :
            returnVal =  this->pimpl->SixAxisFTSensorsMeasurements.size();
            break;
        case ACCELEROMETER : 
            returnVal =  this->pimpl->AccelerometerMeasurements.size();
            break;
        case GYROSCOPE : 
            returnVal =  this->pimpl->GyroscopeMeasurements.size();
        default :
            returnVal = 0;
    }
    return(returnVal);
}

    
}
