/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */



#include <vector>
#include <map>

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Sensors/Sensors.h>

#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>

#include <iDynTree/Core/VectorDynSize.h>

#include <cassert>
#include <iostream>

namespace iDynTree {

Sensor::~Sensor()
{

}

JointSensor::~JointSensor()
{

}

LinkSensor::~LinkSensor()
{

}

///////////////////////////////////////////////////////////////////////////////
///// SensorList
///////////////////////////////////////////////////////////////////////////////

struct SensorsList::SensorsListPimpl
{
    std::vector<std::vector<Sensor *> > allSensors;
    typedef std::map<std::string, std::ptrdiff_t> SensorNameToIndexMap;
    std::vector<SensorNameToIndexMap> sensorsNameToIndex;
};

SensorsList::SensorsList():
    pimpl(new SensorsListPimpl())
{
    //resize datastructures;
    this->pimpl->allSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->sensorsNameToIndex.resize(NR_OF_SENSOR_TYPES);
}

void SensorsList::constructor(const SensorsList& other)
{
    this->pimpl = new SensorsListPimpl();
    this->pimpl->allSensors.resize(NR_OF_SENSOR_TYPES,std::vector<Sensor *>(0));
    this->pimpl->sensorsNameToIndex.resize(NR_OF_SENSOR_TYPES);
    for(int sens_type = 0; sens_type < NR_OF_SENSOR_TYPES; sens_type++ )
    {
        for(std::size_t sens = 0; sens < other.getNrOfSensors((SensorType)sens_type); sens++ )
        {
            this->pimpl->allSensors[sens_type].push_back(other.pimpl->allSensors[sens_type][sens]->clone());
            std::string sensor_name = other.getSensor((SensorType)sens_type,sens)->getName();
            this->pimpl->sensorsNameToIndex[sens_type].insert(std::pair<std::string,std::ptrdiff_t>(sensor_name,sens));
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
        for(size_t sensor_index = 0;
            sensor_index < this->pimpl->allSensors[sensor_type].size(); sensor_index++ )
        {
            delete this->pimpl->allSensors[sensor_type][sensor_index];
        }
    }
    this->pimpl->allSensors.resize(0);
    this->pimpl->sensorsNameToIndex.resize(0);

    delete this->pimpl;
    this->pimpl = 0;
}


SensorsList::~SensorsList()
{
    this->destructor();
}

std::ptrdiff_t  SensorsList::addSensor(const Sensor& sensor)
 {
    Sensor *newSensor = sensor.clone();
    if( ! newSensor->isValid() )
    {
         std::cerr << "[ERR] SensorsTree::addSensor error : sensor  " << sensor.getName()
                       << "  isValid() method returns false" << std::endl;
         delete newSensor;
         return -1;
    }

    if( !(newSensor->getSensorType() >= 0) )
    {
         std::cerr << "[ERR] SensorsTree::addSensor error : sensor  " << sensor.getName()
                       << " has an unknown sensor type " << newSensor->getSensorType() << std::endl;
         delete newSensor;
             return -1;
    }

    this->pimpl->allSensors[newSensor->getSensorType()].push_back(newSensor);
    std::size_t new_index = this->pimpl->allSensors[newSensor->getSensorType()].size()-1;
    this->pimpl->sensorsNameToIndex[newSensor->getSensorType()].insert(std::pair<std::string,size_t>(newSensor->getName(),new_index));

    return new_index;
}

bool SensorsList::getSerialization(const SensorType& sensor_type, std::vector< std::string >& serializaton)
{
    serializaton.resize(0);
    for (size_t sensor_index = 0;
        sensor_index < this->pimpl->allSensors[sensor_type].size(); sensor_index++ )
    {
        std::string sensorName = this->pimpl->allSensors[sensor_type][sensor_index]->getName();
        serializaton.push_back(sensorName);
    }

    return true;
}

bool SensorsList::setSerialization(const SensorType& sensor_type,
                                   const std::vector< std::string >& serializaton)
{
    if( serializaton.size() != this->getNrOfSensors(sensor_type) )
    {
         std::cerr << "[ERROR] SensorsTree::setSerialization error : wrong size of serializaton vector" << std::endl;
         return false;
    }

    std::vector<Sensor *> newVecSensors(serializaton.size());

    for(size_t i=0; i < serializaton.size(); i++ )
    {
        std::ptrdiff_t oldSensIndex = getSensorIndex(sensor_type,serializaton[i]);
        if( oldSensIndex == -1 )
        {
            std::cerr << "[ERROR] SensorsTree::setSerialization error : sensor " << serializaton[i] << " not found in sensor list." << std::endl;
            return false;
        }
        newVecSensors[i] = this->getSensor(sensor_type,oldSensIndex);
    }

    this->pimpl->allSensors[sensor_type] = newVecSensors;

    return true;
}

std::size_t SensorsList::getNrOfSensors(const SensorType & sensor_type) const
{
    return this->pimpl->allSensors[sensor_type].size();
}

bool SensorsList::getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, std::ptrdiff_t & sensor_index) const
{
    SensorsListPimpl::SensorNameToIndexMap::const_iterator it;
    it = this->pimpl->sensorsNameToIndex[sensor_type].find(_sensor_name);
    if( it == this->pimpl->sensorsNameToIndex[sensor_type].end() )
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

bool SensorsList::getSensorIndex(const SensorType & sensor_type, const std::string & _sensor_name, unsigned int & sensor_index) const
{
    std::ptrdiff_t sensor_index_full ;
    bool ret = this->getSensorIndex(sensor_type, _sensor_name, sensor_index_full);
    if (ret) 
    {
        sensor_index = static_cast<unsigned int>(sensor_index_full);
    }
    return ret;
}


std::ptrdiff_t SensorsList::getSensorIndex(const SensorType& sensor_type, const std::string& _sensor_name) const
{
    std::ptrdiff_t retVal;
    bool ok = getSensorIndex(sensor_type,_sensor_name,retVal);

    if( !ok )
    {
        retVal = -1;
    }

    return retVal;
}


Sensor* SensorsList::getSensor(const SensorType& sensor_type, std::ptrdiff_t sensor_index) const
{
    if( sensor_index < (int)getNrOfSensors(sensor_type) && sensor_index >= 0 )
    {
        return this->pimpl->allSensors[sensor_type][sensor_index];
    }
    else
    {
        std::cerr << "[ERROR] getSensor did not find sensor "
                  << sensor_index << " of type " << sensor_type
                  << std::endl;
        return 0;
    }
}

size_t SensorsList::getSizeOfAllSensorsMeasurements() const
{
    size_t res = 0;
    for(int i=(int)SIX_AXIS_FORCE_TORQUE; i< NR_OF_SENSOR_TYPES; i++)
    {
        SensorType type = (SensorType)i;
        res += getSensorTypeSize(type)*getNrOfSensors(type);
    }
    return res;
}

bool SensorsList::isConsistent(const Model& model) const
{
    bool isConsistent = true;
    for (SensorsList::ConstIterator it = this->allSensorsIterator();
         it.isValid(); ++it)
    {
        bool isCurrentSensorConsistent = (*it)->isConsistent(model);
        isConsistent = isConsistent && isCurrentSensorConsistent;
    }
    return isConsistent;
}

bool SensorsList::removeAllSensorsOfType(const iDynTree::SensorType &sensor_type)
{
    //data is cloned when sensors are added. We have to release the associated memory
    for (std::vector<Sensor*>::iterator it = this->pimpl->allSensors[sensor_type].begin();
         it != this->pimpl->allSensors[sensor_type].end(); ++it) {
        delete *it;
    }
    this->pimpl->allSensors[sensor_type].clear();
    this->pimpl->sensorsNameToIndex[sensor_type].clear();
    return true;
}

    bool SensorsList::removeSensor(const SensorType & sensor_type, const std::ptrdiff_t sensor_index)
    {
        std::vector<Sensor*>& typeVector = this->pimpl->allSensors[sensor_type];
        if (sensor_index >= static_cast<std::ptrdiff_t>(typeVector.size())) {
            return false;
        }
        Sensor *s = typeVector[sensor_index];
        typeVector.erase(typeVector.begin() + sensor_index);
        SensorsListPimpl::SensorNameToIndexMap& nameToIndex = this->pimpl->sensorsNameToIndex[sensor_type];
        // We have to rebuild the name->index map as indices have changed
        nameToIndex.clear();
        for (size_t index = 0; index < typeVector.size(); ++index) {
            nameToIndex.insert(SensorsListPimpl::SensorNameToIndexMap::value_type(typeVector[index]->getName(), index));
        }

        delete s;
        return true;
    }

    bool SensorsList::removeSensor(const iDynTree::SensorType &sensor_type, const std::string &_sensor_name)
    {
        std::ptrdiff_t index = getSensorIndex(sensor_type, _sensor_name);
        if (index < 0) return false;
        return removeSensor(sensor_type, index);
    }


    //Iterator implementation
    SensorsList::Iterator SensorsList::allSensorsIterator()
    {
        Iterator iterator(this->pimpl->allSensors);
        return iterator;
    }

    SensorsList::ConstIterator SensorsList::allSensorsIterator() const
    {
        ConstIterator iterator(this->pimpl->allSensors);
        return iterator;
    }

    SensorsList::TypedIterator SensorsList::sensorsIteratorForType(const iDynTree::SensorType &sensor_type)
    {
        TypedIterator iterator(this->pimpl->allSensors[sensor_type]);
        return iterator;
    }

    SensorsList::ConstTypedIterator SensorsList::sensorsIteratorForType(const iDynTree::SensorType &sensor_type) const
    {
        ConstTypedIterator iterator(this->pimpl->allSensors[sensor_type]);
        return iterator;
    }

    SensorsList::TypedIterator::TypedIterator(std::vector<Sensor *> &list)
    : iteratingList(list)
    {
        internalIterator = iteratingList.begin();
    }

    SensorsList::TypedIterator& SensorsList::TypedIterator::operator++()
    {
        ++internalIterator;
        return *this;
    }
    SensorsList::TypedIterator SensorsList::TypedIterator::operator++(int)
    {
        SensorsList::TypedIterator previous(*this);
        this->operator++();
        return previous;
    }

    bool SensorsList::TypedIterator::operator==(const SensorsList::TypedIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    bool SensorsList::TypedIterator::operator==(const SensorsList::ConstTypedIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    SensorsList::TypedIterator::reference SensorsList::TypedIterator::operator*() const
    {
        return *internalIterator;
    }
    SensorsList::TypedIterator::pointer SensorsList::TypedIterator::operator->() const
    {
        return internalIterator.operator->();
    }

    bool SensorsList::TypedIterator::isValid() const
    {
        return internalIterator >= iteratingList.begin() &&
        internalIterator < iteratingList.end();
    }

    SensorsList::ConstTypedIterator::ConstTypedIterator(std::vector<Sensor *> &list)
    : iteratingList(list)
    {
        constructor();
    }

    SensorsList::ConstTypedIterator::ConstTypedIterator(const TypedIterator&it)
    : iteratingList(it.iteratingList)
    {
        constructor();
    }

    void SensorsList::ConstTypedIterator::constructor()
    {
        internalIterator = iteratingList.begin();
    }

    SensorsList::ConstTypedIterator& SensorsList::ConstTypedIterator::operator++()
    {
        ++internalIterator;
        return *this;
    }
    SensorsList::ConstTypedIterator SensorsList::ConstTypedIterator::operator++(int)
    {
        SensorsList::ConstTypedIterator previous(*this);
        this->operator++();
        return previous;
    }

    bool SensorsList::ConstTypedIterator::operator==(const SensorsList::ConstTypedIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    bool SensorsList::ConstTypedIterator::operator==(const SensorsList::TypedIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    SensorsList::ConstTypedIterator::reference SensorsList::ConstTypedIterator::operator*() const
    {
        return internalIterator.operator*();
    }
    SensorsList::ConstTypedIterator::pointer SensorsList::ConstTypedIterator::operator->() const
    {
        return internalIterator.operator->();
    }

    bool SensorsList::ConstTypedIterator::isValid() const
    {
        return internalIterator >= iteratingList.begin() &&
        internalIterator < iteratingList.end();
    }

    SensorsList::Iterator::Iterator(std::vector< std::vector<Sensor *> >&list)
    : iteratingList(list)
    {
        //Start from the beginning
        externalIterator = iteratingList.begin();
        //While external list is empty, skip to the next one
        while (externalIterator != iteratingList.end()
               && externalIterator->empty()) {
            ++externalIterator;
        }
        //if the iterator is still valid assign the internal
        //otherwise the iterator itself is no longer valid
        if (externalIterator != iteratingList.end()) {
            internalIterator = externalIterator->begin();
        }
    }

    SensorsList::Iterator& SensorsList::Iterator::operator++()
    {
        ++internalIterator;
        if (internalIterator >= externalIterator->end()) {
            //end of inner list.
            //Move to next list
            do {
                ++externalIterator;
            } while (externalIterator != iteratingList.end() &&
                     externalIterator->empty());

            if (externalIterator != iteratingList.end()) {
                internalIterator = externalIterator->begin();
            }
        }
        return *this;
    }
    SensorsList::Iterator SensorsList::Iterator::operator++(int)
    {
        SensorsList::Iterator previous(*this);
        this->operator++();
        return previous;
    }

    bool SensorsList::Iterator::operator==(const SensorsList::Iterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    bool SensorsList::Iterator::operator==(const SensorsList::ConstIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    SensorsList::Iterator::reference SensorsList::Iterator::operator*() const
    {
        return *internalIterator;
    }
    SensorsList::Iterator::pointer SensorsList::Iterator::operator->() const
    {
        return internalIterator.operator->();
    }

    bool SensorsList::Iterator::isValid() const
    {
        return
        //Check for external list consistency
        externalIterator >= iteratingList.begin() &&
        externalIterator < iteratingList.end() &&
        //Check for internal list consistency
        internalIterator >= externalIterator->begin() &&
        internalIterator < externalIterator->end();
    }

    SensorsList::ConstIterator::ConstIterator(std::vector< std::vector<Sensor *> >&list)
    : iteratingList(list)
    {
        constructor();
    }

    SensorsList::ConstIterator::ConstIterator(const SensorsList::Iterator& it)
    : iteratingList(it.iteratingList)
    {
        constructor();
    }

    void SensorsList::ConstIterator::constructor()
    {
        //Start from the beginning
        externalIterator = iteratingList.begin();
        //While external list is empty, skip to the next one
        while (externalIterator != iteratingList.end() &&
               externalIterator->empty()) {
            ++externalIterator;
        }
        //if the iterator is still valid assign the internal
        //otherwise the iterator itself is no longer valid
        if (externalIterator != iteratingList.end()) {
            internalIterator = externalIterator->begin();
        }
    }

    SensorsList::ConstIterator& SensorsList::ConstIterator::operator++()
    {
        ++internalIterator;
        if (internalIterator >= externalIterator->end()) {
            //end of inner list.
            //Move to next list
            do {
                ++externalIterator;
            } while (externalIterator != iteratingList.end() &&
                     externalIterator->empty());

            if (externalIterator != iteratingList.end()) {
                internalIterator = externalIterator->begin();
            }
        }
        return *this;
    }
    SensorsList::ConstIterator SensorsList::ConstIterator::operator++(int)
    {
        SensorsList::ConstIterator previous(*this);
        this->operator++();
        return previous;
    }

    bool SensorsList::ConstIterator::operator==(const SensorsList::ConstIterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    bool SensorsList::ConstIterator::operator==(const SensorsList::Iterator&s) const
    {
        return internalIterator == s.internalIterator;
    }

    SensorsList::ConstIterator::reference SensorsList::ConstIterator::operator*() const
    {
        return *internalIterator;
    }
    SensorsList::ConstIterator::pointer SensorsList::ConstIterator::operator->() const
    {
        return internalIterator.operator->();
    }

    bool SensorsList::ConstIterator::isValid() const
    {
        return
        //Check for external list consistency
        externalIterator >= iteratingList.begin() &&
        externalIterator < iteratingList.end() &&
        //Check for internal list consistency
        internalIterator >= externalIterator->begin() &&
        internalIterator < externalIterator->end();
    }

///////////////////////////////////////////////////////////////////////////////
///// LinkSensor
///////////////////////////////////////////////////////////////////////////////

bool LinkSensor::isConsistent(const Model& model) const
{
    LinkIndex lnkIdxInModel = model.getLinkIndex(this->getParentLink());

    if (lnkIdxInModel == LINK_INVALID_INDEX)
    {
        std::cerr << "[ERROR] Sensor " << this->getName() << " is not consistent because the link "
                  << this->getParentLink() << " does not exist in the specified model" << std::endl;
        return false;
    }

    if (lnkIdxInModel != this->getParentLinkIndex())
    {
        std::cerr << "[ERROR] Sensor " << this->getName() << " is not consistent because it is attached to link "
                  << this->getParentLink() << " that has index " << lnkIdxInModel << " in the model, while the sensor "
                  << " has it saved with link index " << this->getParentLinkIndex() << std::endl;
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
///// JointSensor
///////////////////////////////////////////////////////////////////////////////

bool JointSensor::isConsistent(const Model& model) const
{
    LinkIndex jntIdxInModel = model.getJointIndex(this->getParentJoint());

    if (jntIdxInModel == JOINT_INVALID_INDEX)
    {
        std::cerr << "[ERROR] Sensor " << this->getName() << " is not consistent because the joint "
                  << this->getParentJoint() << " does not exist in the specified model" << std::endl;
        return false;
    }

    if (jntIdxInModel != this->getParentJointIndex())
    {
        std::cerr << "[ERROR] Sensor " << this->getName() << " is not consistent because it is attached to joint "
                  << this->getParentJoint() << " that has index " << jntIdxInModel << " in the model, while the sensor "
                  << " has it saved with link index " << this->getParentJointIndex() << std::endl;
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
///// SensorMeasurements
///////////////////////////////////////////////////////////////////////////////

struct SensorsMeasurements::SensorsMeasurementsPrivateAttributes
{
    std::vector< iDynTree::Wrench > SixAxisFTSensorsMeasurements;
    std::vector< iDynTree::AngVelocity > GyroscopeMeasurements;
    std::vector< iDynTree::LinAcceleration> AccelerometerMeasurements;
    std::vector< Vector3 > ThreeAxisAngularAccelerometerMeasurements;
    std::vector< Vector3 > ThreeAxisForceTorqueContactMeasurements;

};


 SensorsMeasurements::SensorsMeasurements() : pimpl(new SensorsMeasurementsPrivateAttributes)
{

}

SensorsMeasurements::SensorsMeasurements(const SensorsList &sensorsList)
{
    this->pimpl = new SensorsMeasurementsPrivateAttributes;
    this->pimpl->SixAxisFTSensorsMeasurements.resize(sensorsList.getNrOfSensors(SIX_AXIS_FORCE_TORQUE));
    this->pimpl->AccelerometerMeasurements.resize(sensorsList.getNrOfSensors(ACCELEROMETER));
    this->pimpl->GyroscopeMeasurements.resize(sensorsList.getNrOfSensors(GYROSCOPE));
    this->pimpl->ThreeAxisAngularAccelerometerMeasurements.resize(sensorsList.getNrOfSensors(THREE_AXIS_ANGULAR_ACCELEROMETER));
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

bool SensorsMeasurements::setNrOfSensors(const SensorType& sensor_type, std::size_t nrOfSensors)
{
    Wrench zeroWrench;
    LinAcceleration zeroLinAcc;
    AngVelocity zeroAngVel;
    bool ret = true;
    switch (sensor_type)
    {
        case SIX_AXIS_FORCE_TORQUE :
            zeroWrench.zero();
            this->pimpl->SixAxisFTSensorsMeasurements.resize(nrOfSensors,zeroWrench);
            break;
        case ACCELEROMETER :
            zeroLinAcc.zero();
            this->pimpl->AccelerometerMeasurements.resize(nrOfSensors,zeroLinAcc);
            break;
        case GYROSCOPE :
            zeroAngVel.zero();
            this->pimpl->GyroscopeMeasurements.resize(nrOfSensors,zeroAngVel);
            break;
        case THREE_AXIS_ANGULAR_ACCELEROMETER:
        {
            Vector3 zeroAngAcc;
            zeroAngAcc.zero();
            this->pimpl->ThreeAxisAngularAccelerometerMeasurements.resize(nrOfSensors, zeroAngAcc);
        }
            break;
        case THREE_AXIS_FORCE_TORQUE_CONTACT:
        {
            Vector3 zeroFT;
            zeroFT.zero();
            this->pimpl->ThreeAxisForceTorqueContactMeasurements.resize(nrOfSensors, zeroFT);
        }
            break;
        default :
            ret = false;
            break;
    }

    return ret;
}

bool SensorsMeasurements::resize(const SensorsList &sensorsList)
{
    Wrench zeroWrench;zeroWrench.zero();
    LinAcceleration zeroLinAcc;zeroLinAcc.zero();
    AngVelocity zeroAngVel;zeroAngVel.zero();
    Vector3  zeroVec3; zeroVec3.zero();

    this->pimpl->SixAxisFTSensorsMeasurements.resize(sensorsList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),zeroWrench);
    this->pimpl->AccelerometerMeasurements.resize(sensorsList.getNrOfSensors(iDynTree::ACCELEROMETER),zeroLinAcc);
    this->pimpl->GyroscopeMeasurements.resize(sensorsList.getNrOfSensors(iDynTree::GYROSCOPE),zeroAngVel);
    this->pimpl->ThreeAxisAngularAccelerometerMeasurements.resize(sensorsList.getNrOfSensors(iDynTree::THREE_AXIS_ANGULAR_ACCELEROMETER),zeroAngVel);
    this->pimpl->ThreeAxisForceTorqueContactMeasurements.resize(sensorsList.getNrOfSensors(THREE_AXIS_FORCE_TORQUE_CONTACT), zeroVec3);

    return true;
}

bool SensorsMeasurements::toVector(VectorDynSize & measurementVector) const
{
    std::size_t itr;
    LinAcceleration thisLinAcc;
    AngVelocity thisAngVel;
    Wrench thisWrench;
    std::size_t numFT = this->pimpl->SixAxisFTSensorsMeasurements.size();
    std::size_t numAcc = this->pimpl->AccelerometerMeasurements.size();
    std::size_t numGyro = this->pimpl->GyroscopeMeasurements.size();
    std::size_t numThreeAxisAngularAcc = this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size();
    std::size_t numThreeAxisFT = this->pimpl->ThreeAxisForceTorqueContactMeasurements.size();

    bool ok = true;

    measurementVector.resize(6*numFT + 3*numAcc + 3*numGyro + 3*numThreeAxisAngularAcc + 3*numThreeAxisFT);

    for(itr = 0; itr<numFT; itr++)
    {
        thisWrench = this->pimpl->SixAxisFTSensorsMeasurements.at(itr);

        ok && measurementVector.setVal(6*itr, thisWrench.getVal(0));
        ok && measurementVector.setVal(6*itr+1,thisWrench.getVal(1));
        ok && measurementVector.setVal(6*itr+2,thisWrench.getVal(2));
        ok && measurementVector.setVal(6*itr+3,thisWrench.getVal(3));
        ok && measurementVector.setVal(6*itr+4,thisWrench.getVal(4));
        ok && measurementVector.setVal(6*itr+5,thisWrench.getVal(5));
    }
    for(itr = 0; itr<numAcc; itr++)
    {
        thisLinAcc =  this->pimpl->AccelerometerMeasurements.at(itr);
        ok && measurementVector.setVal(6*numFT + 3*itr,thisLinAcc.getVal(0));
        ok && measurementVector.setVal(6*numFT + 3*itr+1,thisLinAcc.getVal(1));
        ok && measurementVector.setVal(6*numFT + 3*itr+2,thisLinAcc.getVal(2));
    }
    for(itr = 0; itr<numGyro; itr++)
    {
        thisAngVel = this->pimpl->GyroscopeMeasurements.at(itr);
        ok && measurementVector.setVal(6*numFT + 3*numAcc + 3*itr,thisAngVel.getVal(0));
        ok && measurementVector.setVal(6*numFT + 3*numAcc + 3*itr+1,thisAngVel.getVal(1));
        ok && measurementVector.setVal(6*numFT + 3*numAcc + 3*itr+2,thisAngVel.getVal(2));
    }
    for(itr = 0; itr<numThreeAxisAngularAcc; itr++)
    {
        std::ptrdiff_t offset = 6*numFT + 3*numAcc + 3*numGyro + 3*itr;
        Vector3 thisAngAcc = this->pimpl->ThreeAxisAngularAccelerometerMeasurements.at(itr);
        ok && measurementVector.setVal(offset,   thisAngAcc.getVal(0));
        ok && measurementVector.setVal(offset+1, thisAngAcc.getVal(1));
        ok && measurementVector.setVal(offset+2, thisAngAcc.getVal(2));
    }
    for(itr = 0; itr<numThreeAxisFT; itr++)
    {
        std::ptrdiff_t offset = 6*numFT + 3*numAcc + 3*numGyro + 3*numThreeAxisAngularAcc + 3*itr;
        Vector3 thisThreeAxisFT = this->pimpl->ThreeAxisForceTorqueContactMeasurements.at(itr);
        ok && measurementVector.setVal(offset,   thisThreeAxisFT.getVal(0));
        ok && measurementVector.setVal(offset+1, thisThreeAxisFT.getVal(1));
        ok && measurementVector.setVal(offset+2, thisThreeAxisFT.getVal(2));
    }

    return ok;
}


bool SensorsMeasurements::setMeasurement(const SensorType& sensor_type,
                                         const std::ptrdiff_t& sensor_index,
                                         const iDynTree::Wrench &measurement )
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->SixAxisFTSensorsMeasurements.size() )
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
                                         const std::ptrdiff_t& sensor_index,
                                         const Vector3& measurement)
{
    if( sensor_type == ACCELEROMETER )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->AccelerometerMeasurements.size() )
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

    if( sensor_type == GYROSCOPE )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->GyroscopeMeasurements.size() )
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

    if (sensor_type == THREE_AXIS_ANGULAR_ACCELEROMETER)
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size() )
        {
            this->pimpl->ThreeAxisAngularAccelerometerMeasurements[sensor_index] = measurement;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] setMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size() << std::endl;
            return false;
        }
    }

    if (sensor_type == THREE_AXIS_FORCE_TORQUE_CONTACT)
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->ThreeAxisForceTorqueContactMeasurements.size() )
        {
            this->pimpl->ThreeAxisForceTorqueContactMeasurements[sensor_index] = measurement;
            return true;
        }
        else
        {
            std::cerr << "[ERROR] setMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->ThreeAxisForceTorqueContactMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

bool SensorsMeasurements::getMeasurement(const SensorType& sensor_type,
                                         const std::ptrdiff_t& sensor_index,
                                         Wrench& measurement) const
{
    if( sensor_type == SIX_AXIS_FORCE_TORQUE )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->SixAxisFTSensorsMeasurements.size() )
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

bool SensorsMeasurements::getMeasurement(const SensorType &sensor_type, const std::ptrdiff_t &sensor_index,
                                         Vector3 &measurement) const
{
    if( sensor_type == ACCELEROMETER )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->AccelerometerMeasurements.size() )
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

    if( sensor_type == GYROSCOPE )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->GyroscopeMeasurements.size() )
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

    if( sensor_type == THREE_AXIS_ANGULAR_ACCELEROMETER )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size() )
        {
            measurement = this->pimpl->ThreeAxisAngularAccelerometerMeasurements[sensor_index];
            return true;
        }
        else
        {
            std::cerr << "[ERROR] getMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size() << std::endl;
            return false;
        }
    }

    if( sensor_type == THREE_AXIS_FORCE_TORQUE_CONTACT )
    {
        if( static_cast<std::size_t>(sensor_index) < this->pimpl->ThreeAxisForceTorqueContactMeasurements.size() )
        {
            measurement = this->pimpl->ThreeAxisForceTorqueContactMeasurements[sensor_index];
            return true;
        }
        else
        {
            std::cerr << "[ERROR] getMeasurement failed: sensor_index " << sensor_index
                      << "is out of bounds, because nrOfSensors is "
                      << this->pimpl->ThreeAxisForceTorqueContactMeasurements.size() << std::endl;
            return false;
        }
    }

    return false;
}

std::size_t SensorsMeasurements::getNrOfSensors(const SensorType& sensor_type) const
{
    std::size_t returnVal = 0;
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
            break;
        case THREE_AXIS_ANGULAR_ACCELEROMETER :
            returnVal =  this->pimpl->ThreeAxisAngularAccelerometerMeasurements.size();
            break;
        case THREE_AXIS_FORCE_TORQUE_CONTACT :
            returnVal =  this->pimpl->ThreeAxisForceTorqueContactMeasurements.size();
            break;
        default :
            returnVal = 0;
    }
    return(returnVal);
}

size_t SensorsMeasurements::getSizeOfAllSensorsMeasurements() const
{
    size_t res = 0;
    for(int i=(int)SIX_AXIS_FORCE_TORQUE; i< NR_OF_SENSOR_TYPES; i++)
    {
        SensorType type = (SensorType)i;
        res += getSensorTypeSize(type)*getNrOfSensors(type);
    }
    return res;
}

}
