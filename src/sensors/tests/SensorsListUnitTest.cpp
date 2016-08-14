#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Sensors/Sensors.h>
#include <iDynTree/Sensors/AccelerometerSensor.h>
#include <iDynTree/Sensors/SixAxisFTSensor.h>

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

}


int main()
{
    checkList();
    return EXIT_SUCCESS;
}
