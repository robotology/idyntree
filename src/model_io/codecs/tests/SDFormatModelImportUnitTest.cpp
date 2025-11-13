// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

#include <iDynTree/TestUtils.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

unsigned int getNrOfVisuals(const iDynTree::Model &model)
{
    unsigned int nrOfVisuals = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfVisuals += model.visualSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfVisuals;
}

unsigned int getNrOfCollisions(const iDynTree::Model &model)
{
    unsigned int nrOfCollisions = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index)
    {
        nrOfCollisions +=
            model.collisionSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfCollisions;
}

void checkSDFormat(std::string fileName, unsigned int expectedNrOfLinks,
                   unsigned int expectedNrOfJoints,
                   unsigned int expectedNrOfDOFs,
                   std::string expectedDefaultBase)
{
    ModelLoader loader;
    // Explicitly specify "sdf" filetype to ensure SDF parser is used
    bool ok = loader.loadModelFromFile(fileName, "sdf");

    if (!ok)
    {
        std::cerr << "Failed to load SDF file: " << fileName << std::endl;
        ASSERT_IS_TRUE(false);
        return;
    }

    Model model = loader.model();

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::cerr << model.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), expectedNrOfDOFs);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),
                        expectedDefaultBase);

    // Verify that links have valid inertial properties
    for (LinkIndex linkIdx = 0; linkIdx < model.getNrOfLinks(); ++linkIdx)
    {
        const Link *link = model.getLink(linkIdx);
        const SpatialInertia &inertia = link->getInertia();

        // Check that mass is positive
        ASSERT_IS_TRUE(inertia.getMass() > 0.0);

        std::cerr << "Link " << model.getLinkName(linkIdx)
                  << " mass: " << inertia.getMass() << " kg" << std::endl;
    }

    // Verify joints have correct parent-child relationships
    for (JointIndex jointIdx = 0; jointIdx < model.getNrOfJoints(); ++jointIdx)
    {
        IJointConstPtr joint = model.getJoint(jointIdx);

        LinkIndex link1 = joint->getFirstAttachedLink();
        LinkIndex link2 = joint->getSecondAttachedLink();

        ASSERT_IS_TRUE(link1 >= 0 && link1 < (LinkIndex)model.getNrOfLinks());
        ASSERT_IS_TRUE(link2 >= 0 && link2 < (LinkIndex)model.getNrOfLinks());

        std::cerr << "Joint " << model.getJointName(jointIdx) << ": "
                  << model.getLinkName(link1) << " -> " << model.getLinkName(link2)
                  << " (DOFs: " << joint->getNrOfDOFs() << ")" << std::endl;
    }
}

void checkSDFormatWithAutoDetect(std::string fileName,
                                 unsigned int expectedNrOfLinks,
                                 unsigned int expectedNrOfJoints,
                                 unsigned int expectedNrOfDOFs,
                                 std::string expectedDefaultBase)
{
    ModelLoader loader;
    // Let the loader auto-detect file type from extension
    bool ok = loader.loadModelFromFile(fileName);

    if (!ok)
    {
        std::cerr << "Failed to auto-detect and load SDF file: " << fileName
                  << std::endl;
        ASSERT_IS_TRUE(false);
        return;
    }

    Model model = loader.model();

    std::cerr << "Model auto-loaded from " << fileName << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), expectedNrOfDOFs);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),
                        expectedDefaultBase);
}

int main()
{
#ifdef IDYNTREE_USES_SDFORMAT
    // Test with simple_model.sdf (2 links, 1 revolute joint)
    std::cout << "\nTesting simple_model.sdf" << std::endl;
    checkSDFormat(getAbsModelPath("simple_model.sdf"),
                  2, // 2 links: base_link, link1
                  1, // 1 joint: joint1 (revolute)
                  1, // 1 DOF
                  "base_link");

    // Test auto-detection with .sdf extension
    std::cout << "\nTesting auto-detection with .sdf extension"
              << std::endl;
    checkSDFormatWithAutoDetect(getAbsModelPath("simple_model.sdf"), 2, 1, 1,
                                "base_link");

#else
    std::cout << "SDFormat support not enabled, skipping tests" << std::endl;
    std::cout << "Rebuild with IDYNTREE_USES_SDFORMAT=ON to enable SDFormat tests"
              << std::endl;
#endif

    return EXIT_SUCCESS;
}
