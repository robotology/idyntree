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
#include <iDynTree/ModelIO/ModelLoader.h>

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

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::cerr << model.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(),expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),expectedNrOfFrames);
    ASSERT_EQUAL_STRING(model.getLinkName(model.getDefaultBaseLink()),expectedDefaultBase);

    checkParsingOfDofsFromURDF(fileName,expectedNrOfDOFs);

    // Check that the copy constructor works fine
    Model modelCopyConstruced = model;

    std::cerr << "Model copy constructed from " << fileName << std::endl;
    std::cerr << modelCopyConstruced.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfDOFs(),expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(modelCopyConstruced.getNrOfFrames(),expectedNrOfFrames);
    ASSERT_EQUAL_STRING(modelCopyConstruced.getLinkName(modelCopyConstruced.getDefaultBaseLink()),expectedDefaultBase);

    // Check that the copy assignent works fine
    Model modelCopyAssigned;

    modelCopyAssigned = model;

    std::cerr << "Model copy assigned from " << fileName << std::endl;
    std::cerr << modelCopyAssigned.toString() << std::endl;


    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfDOFs(),expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(modelCopyAssigned.getNrOfFrames(),expectedNrOfFrames);
    ASSERT_EQUAL_STRING(modelCopyAssigned.getLinkName(modelCopyAssigned.getDefaultBaseLink()),expectedDefaultBase);
}

void checkModelLoderForURDFFile(std::string urdfFile)
{
    ModelLoader loader;
    bool result = loader.loadModelFromFile(urdfFile);
    ASSERT_IS_TRUE(result && loader.isValid());
}

void checkModelLoaderFromURDFString(std::string urdfString, bool shouldBeCorrect = true)
{
    ModelLoader loader;
    bool result = loader.loadModelFromString(urdfString);
    if (shouldBeCorrect) {
        ASSERT_IS_TRUE(result && loader.isValid());
    } else {
        ASSERT_IS_TRUE(!result && !loader.isValid());
    }

}

void checkLimitsForJointsAreDefined(std::string urdfFileName)
{
    Model model;
    bool ok = modelFromURDF(urdfFileName,model);
    ASSERT_IS_TRUE(ok);
    
    for(JointIndex jnt=0; jnt < model.getNrOfJoints(); jnt++)
    {
        IJointPtr jntPtr = model.getJoint(jnt);
        
        ASSERT_IS_TRUE(jntPtr != 0);
        
        if( jntPtr->getNrOfDOFs() == 1 )
        {
            ASSERT_IS_TRUE(jntPtr->hasPosLimits());
            
            double max = jntPtr->getMaxPosLimit(0);
            double min = jntPtr->getMinPosLimit(0);
            
            ASSERT_IS_TRUE(min <= max);
            ASSERT_IS_TRUE(min > -10e7);
            ASSERT_IS_TRUE(max < 10e7);
        }
    }
}

int main()
{
    checkURDF(getAbsModelPath("/oneLink.urdf"),1,0,0,7,"link1");
    checkURDF(getAbsModelPath("twoLinks.urdf"),2,1,1,6,"link1");
    checkURDF(getAbsModelPath("icub_skin_frames.urdf"),39,38,32,62,"root_link");
    checkURDF(getAbsModelPath("iCubGenova02.urdf"),33,32,26,111,"root_link");

    checkModelLoderForURDFFile(getAbsModelPath("/oneLink.urdf"));
    checkModelLoaderFromURDFString("this is not an xml", false);
    
    checkLimitsForJointsAreDefined(getAbsModelPath("iCubGenova02.urdf"));

    return EXIT_SUCCESS;
}