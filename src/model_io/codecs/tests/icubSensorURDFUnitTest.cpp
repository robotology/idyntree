// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
# include <iDynTree/Sensors.h>
#include "testModels.h"
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/URDFDofsImport.h>

#include <iDynTree/TestUtils.h>
#include <iDynTree/AccelerometerSensor.h>
#include <iDynTree/GyroscopeSensor.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
using namespace iDynTree;

void checkURDF(std::string fileName,
               unsigned int expectedNrOfAccelerometers,
               unsigned int expectedNrOfGyroscopes,
               unsigned int expectedNrOfFTSensors,
               const std::string nameOfASensorFrame
              )
{
    std::cout<<"Tying to load model from URDF"<<std::endl;
    // load URDF model
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    ASSERT_IS_TRUE(ok);
    Model model = loader.model();
    std::cout<<"Model "<<fileName.c_str()<<" created with :"<<model.getNrOfDOFs()<<" DoFs"<<std::endl;

    iDynTree::SensorsList sensorList = loader.sensors();
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

     // The default behavior is to load the sensor frame as additional frames, so we check
     // if at least one sensor frame has been correctly loaded
     ASSERT_IS_TRUE(model.isFrameNameUsed(nameOfASensorFrame));

}



int main()
{
    std::cout<<"iCub Sensor test running:\n";
    checkURDF(getAbsModelPath("/icub_sensorised.urdf"),55,11,6,"root_link_ems_acc_eb5");

    std::cout <<"iCub Sensor test just ran\n";
    return 0;
}
