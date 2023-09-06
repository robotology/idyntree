// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Sensors.h>
#include "testModels.h"

#include <iDynTree/TestUtils.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
using namespace iDynTree;

void checkURDF(std::string fileName,
               unsigned int expectedNrOfAccelerometers,
               unsigned int expectedNrOfGyroscopes,
               unsigned int expectedNrOfFTs)
{
    ModelLoader loader;
    ASSERT_IS_TRUE(loader.loadModelFromFile(fileName));
    iDynTree::SensorsList sensorList = loader.sensors();

    std::cout<<"Sensor list created from URDF. num accel : "<<sensorList.getNrOfSensors(iDynTree::ACCELEROMETER)
    <<", num gyro : "<<sensorList.getNrOfSensors(iDynTree::GYROSCOPE)<<std::endl;

    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::ACCELEROMETER),expectedNrOfAccelerometers);
    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::GYROSCOPE),expectedNrOfGyroscopes);
    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),expectedNrOfFTs);

    // Load the reduced model without any joint (this will clamp all the link in one big link)
    std::vector<std::string> consideredJoints;
    bool ok = loader.loadReducedModelFromFile(fileName,consideredJoints);

    ASSERT_IS_TRUE(ok);

    // The number of gyro and accelerometers (and in general of link sensors) should be converved
    ASSERT_EQUAL_DOUBLE(loader.sensors().getNrOfSensors(iDynTree::ACCELEROMETER),expectedNrOfAccelerometers);
    ASSERT_EQUAL_DOUBLE(loader.sensors().getNrOfSensors(iDynTree::GYROSCOPE),expectedNrOfGyroscopes);
}


int main()
{
    std::cout<<"Generic Sensor test running (one Link):\n";
    checkURDF(getAbsModelPath("oneLink.urdf"),2, 1, 0);

    std::cout<<"Generic Sensor test running (two Link):\n";
    checkURDF(getAbsModelPath("/twoLinks.urdf"),2, 1, 0);

    std::cout<<"Generic Sensor test running (icalibrate):\n";
    checkURDF(getAbsModelPath("/icalibrate.urdf"), 0, 0, 1);

    std::cout <<"Generic Sensor test just ran\n";
    return 0;
}
