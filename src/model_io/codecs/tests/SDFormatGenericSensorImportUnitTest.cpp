// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#include "testModels.h"
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Sensors.h>
#include <iDynTree/SolidShapes.h>

#include <iDynTree/TestUtils.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>

using namespace iDynTree;

void checkSDFormatSensorsAndGeometry(std::string fileName,
                                     unsigned int expectedNrOfAccelerometers,
                                     unsigned int expectedNrOfGyroscopes,
                                     unsigned int expectedNrOfFTs,
                                     unsigned int expectedNrOfVisuals,
                                     unsigned int expectedNrOfCollisions)
{

    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    ASSERT_IS_TRUE(ok);

    const Model &model = loader.model();
    const SensorsList &sensorList = loader.sensors();

    // Test sensors
    std::cout << "Sensors loaded: "
              << sensorList.getNrOfSensors(iDynTree::ACCELEROMETER)
              << " accelerometers, "
              << sensorList.getNrOfSensors(iDynTree::GYROSCOPE) << " gyroscopes, "
              << sensorList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)
              << " FT sensors" << std::endl;

    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::ACCELEROMETER),
                        expectedNrOfAccelerometers);
    ASSERT_EQUAL_DOUBLE(sensorList.getNrOfSensors(iDynTree::GYROSCOPE),
                        expectedNrOfGyroscopes);
    ASSERT_EQUAL_DOUBLE(
        sensorList.getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE),
        expectedNrOfFTs);

    // Test visual geometries
    unsigned int nrOfVisuals = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfVisuals += model.visualSolidShapes().getLinkSolidShapes()[index].size();
    }
    std::cout << "Visual geometries: " << nrOfVisuals << std::endl;
    ASSERT_EQUAL_DOUBLE(nrOfVisuals, expectedNrOfVisuals);

    // Test collision geometries
    unsigned int nrOfCollisions = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfCollisions +=
            model.collisionSolidShapes().getLinkSolidShapes()[index].size();
    }
    std::cout << "Collision geometries: " << nrOfCollisions << std::endl;
    ASSERT_EQUAL_DOUBLE(nrOfCollisions, expectedNrOfCollisions);

    // Verify sensor properties if sensors exist
    if (expectedNrOfAccelerometers > 0)
    {
        const Sensor *acc = sensorList.getSensor(iDynTree::ACCELEROMETER, 0);
        ASSERT_IS_TRUE(acc != nullptr);
        std::cout << "  Accelerometer '" << acc->getName() << "' attached to link '"
                  << dynamic_cast<const LinkSensor *>(acc)->getParentLink() << "'"
                  << std::endl;
    }

    if (expectedNrOfGyroscopes > 0)
    {
        const Sensor *gyro = sensorList.getSensor(iDynTree::GYROSCOPE, 0);
        ASSERT_IS_TRUE(gyro != nullptr);
        std::cout << "  Gyroscope '" << gyro->getName() << "' attached to link '"
                  << dynamic_cast<const LinkSensor *>(gyro)->getParentLink() << "'"
                  << std::endl;
    }

    // Verify geometry properties if geometries exist
    if (expectedNrOfVisuals > 0)
    {
        for (LinkIndex linkIdx = 0; linkIdx < model.getNrOfLinks(); ++linkIdx)
        {
            const std::vector<SolidShape *> &shapes =
                model.visualSolidShapes().getLinkSolidShapes()[linkIdx];
            for (const SolidShape *shape : shapes)
            {
                if (shape->isBox())
                {
                    const Box *box = shape->asBox();
                    std::cout << "  Visual Box on link '" << model.getLinkName(linkIdx)
                              << "': " << box->getX() << "x" << box->getY() << "x"
                              << box->getZ() << std::endl;
                }
                else if (shape->isSphere())
                {
                    const Sphere *sphere = shape->asSphere();
                    std::cout << "  Visual Sphere on link '" << model.getLinkName(linkIdx)
                              << "': radius=" << sphere->getRadius() << std::endl;
                }
                else if (shape->isCylinder())
                {
                    const Cylinder *cylinder = shape->asCylinder();
                    std::cout << "  Visual Cylinder on link '"
                              << model.getLinkName(linkIdx)
                              << "': radius=" << cylinder->getRadius()
                              << ", length=" << cylinder->getLength() << std::endl;
                }
            }
        }
    }

    if (expectedNrOfCollisions > 0)
    {
        for (LinkIndex linkIdx = 0; linkIdx < model.getNrOfLinks(); ++linkIdx)
        {
            const std::vector<SolidShape *> &shapes =
                model.collisionSolidShapes().getLinkSolidShapes()[linkIdx];
            for (const SolidShape *shape : shapes)
            {
                if (shape->isBox())
                {
                    const Box *box = shape->asBox();
                    std::cout << "  Collision Box on link '" << model.getLinkName(linkIdx)
                              << "': " << box->getX() << "x" << box->getY() << "x"
                              << box->getZ() << std::endl;
                }
                else if (shape->isSphere())
                {
                    const Sphere *sphere = shape->asSphere();
                    std::cout << "  Collision Sphere on link '"
                              << model.getLinkName(linkIdx)
                              << "': radius=" << sphere->getRadius() << std::endl;
                }
                else if (shape->isCylinder())
                {
                    const Cylinder *cylinder = shape->asCylinder();
                    std::cout << "  Collision Cylinder on link '"
                              << model.getLinkName(linkIdx)
                              << "': radius=" << cylinder->getRadius()
                              << ", length=" << cylinder->getLength() << std::endl;
                }
            }
        }
    }

    std::cout << "✓ All checks passed for " << fileName << std::endl;
}

int main()
{
#ifdef IDYNTREE_USES_SDFORMAT
    std::cout << "SDFormat Generic Sensor and Geometry import running"
              << std::endl;

    // Test the model with sensors and geometry
    checkSDFormatSensorsAndGeometry(
        getAbsModelPath("model_with_sensors_geometry.sdf"),
        1, // Expected accelerometers (IMU creates both accel and gyro)
        1, // Expected gyroscopes
        0, // Expected FT sensors
        2, // Expected visual geometries (1 box + 1 cylinder)
        2  // Expected collision geometries (1 sphere + 1 cylinder)
    );

    return 0;
#else
    std::cout << "SDFormat support not enabled, skipping test" << std::endl;
    return 0;
#endif
}
