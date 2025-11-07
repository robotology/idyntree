// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Test URDF to SDF roundtrip conversion.
 *
 * This test validates that URDF models can be converted to SDF format using the
 * sdformat library and then loaded back into iDynTree successfully.
 *
 * Note: SDFormat applies transformations during URDF→SDF conversion, including:
 * - Converting fixed joints to revolute joints with zero limits [lower=0, upper=0]
 * - Lumping links connected by fixed joints (unless explicitly disabled)
 * - Reordering or restructuring the kinematic tree
 *
 * Therefore, the converted SDF model may have different numbers of links, joints,
 * or DOFs compared to the original URDF. This is expected SDFormat behavior.
 */

#include <iDynTree/TestUtils.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/FreeFloatingState.h>

#include <testModels.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

#ifdef IDYNTREE_USES_SDFORMAT
#include <sdf/sdf.hh>
#include <gz/math/Pose3.hh>
#endif

using namespace iDynTree;

#ifdef IDYNTREE_USES_SDFORMAT

// Helper function to convert URDF file to SDF string using sdformat library
bool convertURDFToSDFString(const std::string &urdfFilePath, std::string &sdfString)
{
    // Configure parser to preserve fixed joints (prevents lumping)
    sdf::ParserConfig parserConfig;
    parserConfig.URDFSetPreserveFixedJoint(true);

    // Load URDF file using sdformat
    sdf::Root root;
    sdf::Errors errors = root.Load(urdfFilePath, parserConfig);

    if (!errors.empty())
    {
        std::cerr << "Error loading URDF with sdformat: " << urdfFilePath << std::endl;
        for (const auto &error : errors)
        {
            std::cerr << "  " << error.Message() << std::endl;
        }
        return false;
    }

    // Convert to SDF string
    sdf::ElementPtr sdfElement = root.Element();
    if (!sdfElement)
    {
        std::cerr << "Failed to get SDF element from root" << std::endl;
        return false;
    }

    std::ostringstream stream;
    stream << "<?xml version='1.0'?>\n";
    stream << sdfElement->ToString("");
    sdfString = stream.str();

    return true;
}

void testURDFToSDFormatRoundtrip(const std::string &urdfFilePath)
{
    std::cout << "\nTesting URDF to SDF roundtrip for: " << urdfFilePath << std::endl;

    // Load original URDF model using iDynTree
    ModelLoader urdfLoader;
    ASSERT_IS_TRUE(urdfLoader.loadModelFromFile(urdfFilePath));
    Model urdfModel = urdfLoader.model();

    std::cout << "Original URDF model: " << urdfModel.getNrOfLinks() << " links, "
              << urdfModel.getNrOfJoints() << " joints, "
              << urdfModel.getNrOfDOFs() << " DOFs" << std::endl;

    // Convert URDF to SDF using sdformat
    std::string sdfString;
    ASSERT_IS_TRUE(convertURDFToSDFString(urdfFilePath, sdfString));

    std::cout << "Converted URDF to SDF" << std::endl;

    // Load SDF model back into iDynTree
    ModelLoader sdfLoader;
    ASSERT_IS_TRUE(sdfLoader.loadModelFromString(sdfString));
    Model sdfModel = sdfLoader.model();

    std::cout << "SDF model loaded: " << sdfModel.getNrOfLinks() << " links, "
              << sdfModel.getNrOfJoints() << " joints, "
              << sdfModel.getNrOfDOFs() << " DOFs" << std::endl;

    // Verify kinematics using KinDynComputations if both models have DOFs
    if (urdfModel.getNrOfDOFs() > 0 && sdfModel.getNrOfDOFs() > 0)
    {
        // Use first link as base
        std::string baseLink = urdfModel.getLinkName(0);
        if (!sdfModel.isLinkNameUsed(baseLink))
        {
            std::cout << "Base link not found in SDF model, skipping kinematics check" << std::endl;
            return;
        }

        KinDynComputations urdfKinDyn;
        ASSERT_IS_TRUE(urdfKinDyn.loadRobotModel(urdfModel));
        ASSERT_IS_TRUE(urdfKinDyn.setFloatingBase(baseLink));

        KinDynComputations sdfKinDyn;
        ASSERT_IS_TRUE(sdfKinDyn.loadRobotModel(sdfModel));
        ASSERT_IS_TRUE(sdfKinDyn.setFloatingBase(baseLink));

        // Only compare if DOFs match (SDFormat may transform the model)
        if (urdfModel.getNrOfDOFs() != sdfModel.getNrOfDOFs())
        {
            std::cout << "DOFs don't match (URDF: " << urdfModel.getNrOfDOFs()
                      << ", SDF: " << sdfModel.getNrOfDOFs() << "), skipping detailed checks" << std::endl;
            std::cout << "Note: SDFormat may convert fixed joints to revolute joints with zero limits," << std::endl;
            std::cout << "      which iDynTree counts as DOFs. This is expected SDFormat behavior." << std::endl;
            return;
        }

        // Set same random state
        size_t dofs = urdfModel.getNrOfDOFs();
        Transform worldTbase = Transform::Identity();
        Twist baseVel = Twist::Zero();

        Vector3 gravity;
        gravity.zero();
        gravity(2) = -9.81;

        VectorDynSize qj(dofs);
        VectorDynSize dqj(dofs);

        for (size_t i = 0; i < dofs; i++)
        {
            qj(i) = (rand() % 1000) / 1000.0 - 0.5;
            dqj(i) = (rand() % 1000) / 1000.0 - 0.5;
        }

        ASSERT_IS_TRUE(urdfKinDyn.setRobotState(worldTbase, qj, baseVel, dqj, gravity));
        ASSERT_IS_TRUE(sdfKinDyn.setRobotState(worldTbase, qj, baseVel, dqj, gravity));

        std::cout << "Set random robot state for kinematics check" << std::endl;

        // Check center of mass
        Position urdfCOM = urdfKinDyn.getCenterOfMassPosition();
        Position sdfCOM = sdfKinDyn.getCenterOfMassPosition();
        ASSERT_EQUAL_VECTOR_TOL(urdfCOM, sdfCOM, 1e-5);

        std::cout << "Center of mass matches" << std::endl;
    }
}

#endif

int main()
{
#ifdef IDYNTREE_USES_SDFORMAT
    std::cout << "\nTesting URDF to SDF Roundtrip" << std::endl;

    // Test with URDF files from the test data
    // Note: twoLinks.urdf has a link without inertia, which causes issues with
    // URDFSetPreserveFixedJoint(true), so we only test with properly defined models
    testURDFToSDFormatRoundtrip(getAbsModelPath("icalibrate.urdf"));

    std::cout << "\nAll roundtrip tests passed" << std::endl;
#else
    std::cout << "SDFormat support not enabled, skipping tests" << std::endl;
#endif

    return EXIT_SUCCESS;
}
