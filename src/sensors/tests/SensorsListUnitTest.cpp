/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/SixAxisForceTorqueSensor.h>

void checkList()
{

    std::cout << "Checking SensorsList Addition/Removal... " << std::endl;
    using namespace iDynTree;

    Transform dummy(Rotation::Identity(), Position::Zero());

    SixAxisForceTorqueSensor ft1; ft1.setName("ft1");
    ft1.setFirstLinkSensorTransform(0, dummy);
    ft1.setSecondLinkSensorTransform(1, dummy);
    ft1.setAppliedWrenchLink(0);
    SixAxisForceTorqueSensor ft2; ft2.setName("ft2");
    ft2.setFirstLinkSensorTransform(0, dummy);
    ft2.setSecondLinkSensorTransform(1, dummy);
    ft2.setAppliedWrenchLink(0);
    SixAxisForceTorqueSensor ft3; ft3.setName("ft3");
    ft3.setFirstLinkSensorTransform(0, dummy);
    ft3.setSecondLinkSensorTransform(1, dummy);
    ft3.setAppliedWrenchLink(0);

    AccelerometerSensor acc1; acc1.setName("acc1");
    acc1.setLinkSensorTransform(dummy);
    acc1.setParentLinkIndex(0);
    AccelerometerSensor acc2; acc2.setName("acc2");
    acc2.setLinkSensorTransform(dummy);
    acc2.setParentLinkIndex(0);
    AccelerometerSensor acc3; acc3.setName("acc3");
    acc3.setLinkSensorTransform(dummy);
    acc3.setParentLinkIndex(0);

    Sensor* accExpectedOrder[3];
    accExpectedOrder[0] = &acc1;
    accExpectedOrder[1] = &acc3;
    accExpectedOrder[2] = &acc2;

    Sensor* ftsExpectedOrder[3];
    ftsExpectedOrder[0] = &ft3;
    ftsExpectedOrder[1] = &ft1;
    ftsExpectedOrder[2] = &ft2;

    Sensor* allExpectedOrder[6];
    for (int i = 0; i < 3; ++i) {
        allExpectedOrder[i] = ftsExpectedOrder[i];
    }
    for (int i = 0; i < 3; ++i) {
        allExpectedOrder[3 + i] = accExpectedOrder[i];
    }

    SensorsList list;
    for (int i = 0; i < 6; ++i) {
        list.addSensor(*allExpectedOrder[i]);
    }

    ASSERT_IS_TRUE(list.getNrOfSensors(ACCELEROMETER) == 3);
    ASSERT_IS_TRUE(list.getNrOfSensors(SIX_AXIS_FORCE_TORQUE) == 3);

    list.removeAllSensorsOfType(ACCELEROMETER);
    ASSERT_IS_TRUE(list.getNrOfSensors(ACCELEROMETER) == 0);

    list.removeSensor(SIX_AXIS_FORCE_TORQUE, ft3.getName());
    ASSERT_IS_TRUE(list.getNrOfSensors(SIX_AXIS_FORCE_TORQUE) == 2);

    for (SensorsList::typed_iterator it = list.sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE);
         it.isValid(); ++it) {
        ASSERT_IS_TRUE((*it)->getName() != ft3.getName());
    }

}



void checkIterator()
{

    std::cout << "Checking SensorsList Iterator... " << std::endl;
    using namespace iDynTree;

    Transform dummy(Rotation::Identity(), Position::Zero());

    SixAxisForceTorqueSensor ft1; ft1.setName("ft1");
    ft1.setFirstLinkSensorTransform(0, dummy);
    ft1.setSecondLinkSensorTransform(1, dummy);
    ft1.setAppliedWrenchLink(0);
    SixAxisForceTorqueSensor ft2; ft2.setName("ft2");
    ft2.setFirstLinkSensorTransform(0, dummy);
    ft2.setSecondLinkSensorTransform(1, dummy);
    ft2.setAppliedWrenchLink(0);
    SixAxisForceTorqueSensor ft3; ft3.setName("ft3");
    ft3.setFirstLinkSensorTransform(0, dummy);
    ft3.setSecondLinkSensorTransform(1, dummy);
    ft3.setAppliedWrenchLink(0);

    AccelerometerSensor acc1; acc1.setName("acc1");
    acc1.setLinkSensorTransform(dummy);
    acc1.setParentLinkIndex(0);
    AccelerometerSensor acc2; acc2.setName("acc2");
    acc2.setLinkSensorTransform(dummy);
    acc2.setParentLinkIndex(0);
    AccelerometerSensor acc3; acc3.setName("acc3");
    acc3.setLinkSensorTransform(dummy);
    acc3.setParentLinkIndex(0);

    Sensor* accExpectedOrder[3];
    accExpectedOrder[0] = &acc1;
    accExpectedOrder[1] = &acc3;
    accExpectedOrder[2] = &acc2;

    Sensor* ftsExpectedOrder[3];
    ftsExpectedOrder[0] = &ft3;
    ftsExpectedOrder[1] = &ft1;
    ftsExpectedOrder[2] = &ft2;

    Sensor* allExpectedOrder[6];
    for (int i = 0; i < 3; ++i) {
        allExpectedOrder[i] = ftsExpectedOrder[i];
    }
    for (int i = 0; i < 3; ++i) {
        allExpectedOrder[3 + i] = accExpectedOrder[i];
    }

    SensorsList list;
    for (int i = 0; i < 6; ++i) {
        list.addSensor(*allExpectedOrder[i]);
    }

    //Checking ACC iterator
    int checkIndex = 0;
    for (SensorsList::typed_iterator it = list.sensorsIteratorForType(ACCELEROMETER); it.isValid(); ++it, ++checkIndex) {
        //The following cannot be used because sensors are cloned when put inside the SensorList
//        ASSERT_IS_TRUE(*it == accExpectedOrder[checkIndex]);
        //The following cannot be used as operator== does not exist in sensor
//        ASSERT_IS_TRUE(**it == *accExpectedOrder[checkIndex]);
        Sensor *s = *it;
        ASSERT_IS_TRUE(accExpectedOrder[checkIndex]->getName() == s->getName());
        //the following does not work :( I don't know why
    }

    //Checking FTS iterator
    checkIndex = 0;
    for (SensorsList::typed_iterator it = list.sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE); it.isValid(); ++it, ++checkIndex) {
        Sensor *s = *it;
        ASSERT_IS_TRUE(ftsExpectedOrder[checkIndex]->getName() == s->getName());
    }

    //Same checking on const iterator
    checkIndex = 0;
    for (SensorsList::const_typed_iterator it = list.sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE); it.isValid(); ++it, ++checkIndex) {
        Sensor *s = *it;
        ASSERT_IS_TRUE(ftsExpectedOrder[checkIndex]->getName() == s->getName());
    }

    //Checking all sensors iterator
    checkIndex = 0;
    for (SensorsList::iterator it = list.allSensorsIterator(); it.isValid(); ++it, ++checkIndex) {
        Sensor *s = *it;
        ASSERT_IS_TRUE(allExpectedOrder[checkIndex]->getName() == s->getName());
    }

    //Same checking on const iterator
    checkIndex = 0;
    for (SensorsList::const_iterator it = list.allSensorsIterator(); it.isValid(); ++it, ++checkIndex) {
        Sensor *s = *it;
        ASSERT_IS_TRUE(allExpectedOrder[checkIndex]->getName() == s->getName());
    }

    SensorsList::iterator allIterator = list.allSensorsIterator();
    SensorsList::iterator allIterator2 = allIterator;
    SensorsList::const_iterator allIterator3 = allIterator;
    ASSERT_IS_TRUE(allIterator == allIterator2);
    ASSERT_IS_TRUE(allIterator == allIterator3);
    ++allIterator;
    allIterator2++;
    ++allIterator3;
    ASSERT_IS_TRUE(allIterator == allIterator2);
    ASSERT_IS_TRUE(allIterator == allIterator3);

    SensorsList::typed_iterator ftsIterator = list.sensorsIteratorForType(SIX_AXIS_FORCE_TORQUE);
    SensorsList::typed_iterator ftsIterator2 = ftsIterator;
    SensorsList::const_typed_iterator ftsIterator3 = ftsIterator;
    ASSERT_IS_TRUE(ftsIterator == ftsIterator2);
    ASSERT_IS_TRUE(ftsIterator == ftsIterator3);
    ++ftsIterator;
    ftsIterator2++;
    ++ftsIterator3;
    ASSERT_IS_TRUE(ftsIterator == ftsIterator2);
    ASSERT_IS_TRUE(ftsIterator == ftsIterator3);

}


int main()
{
    checkList();
    checkIterator();

    return EXIT_SUCCESS;
}
