/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>
#include <iDynTree/ModelIO/URDFModelExport.h>
#include <iDynTree/ModelIO/URDFDofsImport.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

using namespace iDynTree;


void checkImportExportURDF(std::string fileName)
{
    // Import, export and re-import a URDF file, and check that the key properties of the model are mantained
    Model model;
    bool ok = modelFromURDF(fileName,model);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::cerr << model.toString() << std::endl;

    std::string urdfString;
    ok = URDFStringFromModel(model, urdfString);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model serialized back to xml " << std::endl << urdfString << std::endl;
    Model modelReloaded;

    ok = modelFromURDFString(urdfString, modelReloaded);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model re-loaded " << std::endl;
    std::cerr << modelReloaded.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), modelReloaded.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), modelReloaded.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), modelReloaded.getNrOfDOFs());

    // TODO(traversaro) : uncomment the following line when frames are correctly handled by the exporter
    // ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(), modelReloaded.getNrOfLinks());
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
