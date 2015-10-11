/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFModelImport.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;


void checkURDF(std::string fileName,
                  unsigned int expectedNrOfLinks,
                  unsigned int expectedNrOfJoints,
                  unsigned int expectedNrOfDOFs,
                  std::string expectedDefaultBase)
{
    Model model;
    bool ok = modelFromURDF(fileName,model);
    assert(ok);

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),expectedDefaultBase);
}

int main()
{
    checkURDF(IDYNTREE_TEST_MODELS_PATH"/oneLink.urdf",1,0,0,"link1");
    checkURDF(IDYNTREE_TEST_MODELS_PATH"/twoLinks.urdf",2,1,1,"link1");

    return EXIT_SUCCESS;
}