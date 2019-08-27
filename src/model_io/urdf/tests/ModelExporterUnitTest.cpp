/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>


#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/ModelExporter.h>


#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>

using namespace iDynTree;


void checkImportExportURDF(std::string fileName)
{
    // Import, export and re-import a URDF file, and check that the key properties of the model are mantained
    ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(fileName, "urdf");
    Model model = mdlLoader.model();
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::ifstream original_urdf(fileName);

    if (original_urdf.is_open()) {
        std::cout << original_urdf.rdbuf();
    }

    std::string urdfString;
    ModelExporter mdlExporter;
    ok = mdlExporter.init(model, SensorsList());
    ASSERT_IS_TRUE(ok);
    ok = mdlExporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model serialized back to xml " << std::endl << urdfString << std::endl;

    ModelLoader mdlLoaderReloaded;

    ok = mdlLoaderReloaded.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model modelReloaded = mdlLoaderReloaded.model();

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), modelReloaded.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), modelReloaded.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), modelReloaded.getNrOfDOFs());
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(), modelReloaded.getNrOfFrames());

    // Verify that the link correspond (note that the serialization could have changed)
    for(int lnkIndex=0; lnkIndex < model.getNrOfLinks(); lnkIndex++) {
        LinkIndex lnkIndexInReloaded = modelReloaded.getLinkIndex(model.getLinkName(lnkIndex));
        ASSERT_IS_TRUE(lnkIndexInReloaded != LINK_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getLinkName(lnkIndex) == modelReloaded.getLinkName(lnkIndexInReloaded));
        SpatialInertia inertia = model.getLink(lnkIndex)->getInertia();
        SpatialInertia inertiaReloaded = modelReloaded.getLink(lnkIndexInReloaded)->getInertia();
        std::cerr << "Testing inertia of link " << model.getLinkName(lnkIndex) << std::endl;
        ASSERT_EQUAL_MATRIX(inertia.asMatrix(), inertiaReloaded.asMatrix());
    }

    // Verify that the frame correspond (note that the serialization could have changed)
    for(FrameIndex frameIndex=model.getNrOfLinks(); frameIndex < model.getNrOfFrames(); frameIndex++) {
        FrameIndex frameIndexInReloaded = modelReloaded.getFrameIndex(model.getFrameName(frameIndex));
        ASSERT_IS_TRUE(frameIndexInReloaded != FRAME_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getFrameName(frameIndex) == modelReloaded.getFrameName(frameIndexInReloaded));
        Transform link_H_frame = model.getFrameTransform(frameIndex);
        Transform link_H_frame_reloaded = modelReloaded.getFrameTransform(frameIndexInReloaded);
        ASSERT_EQUAL_MATRIX(link_H_frame.asHomogeneousTransform(), link_H_frame_reloaded.asHomogeneousTransform());
    }

}


int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        if (std::string(IDYNTREE_TESTS_URDFS[mdl]) == "bigman.urdf")
        {
            // walkman model fails this test due to https://github.com/robotology/idyntree/issues/247
            continue;
        }

        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking URDF import/export test on " << urdfFileName << std::endl;
        checkImportExportURDF(urdfFileName);
    }


    return EXIT_SUCCESS;
}
