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
#include <iDynTree/ModelIO/URDFDofsImport.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;


void checkParsingOfDofsFromURDF(std::string fileName,
                                size_t expectedNrOfDOFs)
{
    std::vector<std::string> dofsNameList;
    bool ok = dofsListFromURDF(fileName,dofsNameList);

    ASSERT_EQUAL_DOUBLE(dofsNameList.size(),expectedNrOfDOFs);
}

void checkURDF(std::string fileName,
                  unsigned int expectedNrOfLinks,
                  unsigned int expectedNrOfJoints,
                  unsigned int expectedNrOfDOFs,
                  unsigned int expectedNrOfFrames,
                  std::string expectedDefaultBase)
{
    Model model;
    bool ok = modelFromURDF(fileName,model);
    assert(ok);

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(),expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),expectedNrOfFrames);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),expectedDefaultBase);

    checkParsingOfDofsFromURDF(fileName,expectedNrOfDOFs);
}

int main()
{
    checkURDF(getAbsModelPath("/oneLink.urdf"),1,0,0,4,"link1");
    checkURDF(getAbsModelPath("twoLinks.urdf"),2,1,1,3,"link1");

    return EXIT_SUCCESS;
}