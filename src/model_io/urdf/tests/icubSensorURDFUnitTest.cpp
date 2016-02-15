/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Naveen Kuppuswamy
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

# include <iDynTree/ModelIO/URDFGenericSensorsImport.h>
# include <iDynTree/Sensors/Sensors.h>
#include "testModels.h"
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFDofsImport.h>

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/GyroscopeSensor.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
using namespace iDynTree;

void checkURDF(std::string fileName,
               unsigned int expectedNrOfAccelerometers,
               unsigned int expectedNrOfGyroscopes,
               unsigned int expectedNrOfFTSensors
              )
{
    std::cout<<"Tying to load model from URDF"<<std::endl;
    Model model;
    // load URDF model
    bool ok = modelFromURDF(fileName,model);
    ASSERT_EQUAL_DOUBLE(ok,true);
    std::cout<<"Model "<<fileName.c_str()<<" created with :"<<model.getNrOfDOFs()<<" DoFs"<<std::endl;

    iDynTree::SensorsList sensorList;
    ok = iDynTree::sensorsFromURDF(fileName,sensorList);
    ASSERT_EQUAL_DOUBLE(ok,true);
    std::cout<<"Sensor list created from URDF. num accel : "<<sensorList.getNrOfSensors(ACCELEROMETER)<<", num gyro : "<<sensorList.getNrOfSensors(iDynTree::GYROSCOPE)<<std::endl;

    std::cout<<"Accelerometer Sensors in URDF :\n";
    for (int i  = 0; i<sensorList.getNrOfSensors(ACCELEROMETER);i++)
    {
        AccelerometerSensor *acc = (AccelerometerSensor*)sensorList.getSensor(ACCELEROMETER,i);
        std::cout<<acc->getName()<<"\n";
    }

    std::cout<<"--------------\n";
    std::cout<<"Gyroscope Sensors in URDF :\n";
    for (int i  = 0; i<sensorList.getNrOfSensors(GYROSCOPE);i++)
    {
        GyroscopeSensor *gyr = (GyroscopeSensor*)sensorList.getSensor(GYROSCOPE,i);
        std::cout<<gyr->getName()<<"\n";
    }
     ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::ACCELEROMETER),expectedNrOfAccelerometers);
     ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::GYROSCOPE),expectedNrOfGyroscopes);
     ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),expectedNrOfFTSensors);
}



int main()
{
    std::cout<<"iCub Sensor test running:\n";
    checkURDF(getAbsModelPath("/icub_sensorised.urdf"),55,11,6);

    std::cout <<"iCub Sensor test just ran\n";
    return 0;
}