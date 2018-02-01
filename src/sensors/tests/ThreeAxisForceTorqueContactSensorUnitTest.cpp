/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h>

using namespace iDynTree;

void checkCOPcomputation(const ThreeAxisForceTorqueContactSensor& sensor)
{
    std::vector<Position> loadCellLocations = sensor.getLoadCellLocations();
    VectorDynSize arbitraryLoadCellReadings(loadCellLocations.size());

    for (size_t i=0; i < arbitraryLoadCellReadings.size(); i++)
    {
        arbitraryLoadCellReadings(i) = i/10.0;
    }

    Position copIn3D = sensor.computeCenterOfPressureFromLoadCellMeasurements(arbitraryLoadCellReadings);
    Vector2 cop;
    cop(0) = copIn3D(0);
    cop(1) = copIn3D(1);
    Vector2 copCheck;
    Vector3 measuredFT = sensor.computeThreeAxisForceTorqueFromLoadCellMeasurements(arbitraryLoadCellReadings);
    double zForce  = measuredFT(0);
    double xTorque = measuredFT(1);
    double yTorque = measuredFT(2);
    copCheck(0) = -yTorque/zForce;
    copCheck(1) = xTorque/zForce;

    ASSERT_EQUAL_VECTOR(cop, copCheck);
}


int main()
{
    // Create a simple sensor
    ThreeAxisForceTorqueContactSensor sensor;
    std::vector<Position> loadCellLocations;
    loadCellLocations.clear();
    loadCellLocations.push_back(Position( 1.0,  0.0, 0.0));
    loadCellLocations.push_back(Position( 0.0,  1.0, 0.0));
    loadCellLocations.push_back(Position( -1.0,  0.0, 0.0));
    loadCellLocations.push_back(Position( 0.0, -1.0, 0.0));
    sensor.setLoadCellLocations(loadCellLocations);
    std::vector<Position> loadCellLocationsCheck = sensor.getLoadCellLocations();
    ASSERT_IS_TRUE(loadCellLocations.size() == loadCellLocationsCheck.size());

    checkCOPcomputation(sensor);


    // A bit more complex sensor
    loadCellLocations.clear();
    loadCellLocations.push_back(Position( 1.0,  0.0, 0.0));
    loadCellLocations.push_back(Position( 0.0, -1.0, 0.0));
    loadCellLocations.push_back(Position( 3.0, -5.0, 0.0));
    loadCellLocations.push_back(Position(-7.0,  3.0, 0.0));
    loadCellLocations.push_back(Position( 3.0,  8.0, 0.0));
    loadCellLocations.push_back(Position( -3.0,  2.0, 0.0));
    sensor.setLoadCellLocations(loadCellLocations);
    checkCOPcomputation(sensor);




    return EXIT_SUCCESS;
}
