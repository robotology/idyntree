/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/URDFDofsImport.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

using namespace iDynTree;


void checkParsingOfDofsFromURDF(std::string fileName,
                                size_t expectedNrOfDOFs)
{
    std::vector<std::string> dofsNameList;
    bool ok = dofsListFromURDF(fileName,dofsNameList);
    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_DOUBLE(dofsNameList.size(),expectedNrOfDOFs);
}

unsigned int getNrOfVisuals(const iDynTree::Model& model)
{
    unsigned int nrOfVisuals = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index) {
        nrOfVisuals += model.visualSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfVisuals;
}

unsigned int getNrOfCollisions(const iDynTree::Model& model)
{
    unsigned int nrOfCollisions = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index) {
        nrOfCollisions += model.collisionSolidShapes().getLinkSolidShapes()[index].size();
    }
    return nrOfCollisions;
}

void checkURDF(std::string fileName,
                  unsigned int expectedNrOfLinks,
                  unsigned int expectedNrOfJoints,
                  unsigned int expectedNrOfDOFs,
                  unsigned int expectedNrOfFrames,
                  unsigned int expectedNrOfVisuals,
                  unsigned int expectedNrOfCollisions,
                  std::string expectedDefaultBase)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(fileName);
    Model model = loader.model();
    assert(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::cerr << model.toString() << std::endl;

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),expectedNrOfLinks);
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),expectedNrOfJoints);
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(),expectedNrOfDOFs);
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),expectedNrOfFrames);
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(model), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(model), expectedNrOfCollisions);
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
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(modelCopyConstruced), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(modelCopyConstruced), expectedNrOfCollisions);
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
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(modelCopyAssigned), expectedNrOfVisuals);
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(modelCopyAssigned), expectedNrOfCollisions);
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

void checkLimitsForJointsAreDefined(Model & model)
{

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

void checkLimitsForJointsAreDefinedFromFileName(std::string urdfFileName)
{
    ModelLoader loader;
    bool ok = loader.loadModelFromFile(urdfFileName);
    Model model = loader.model();

    ASSERT_IS_TRUE(ok);
    
    checkLimitsForJointsAreDefined(model);
    
    Model copyConstructedModel = model;
    
    checkLimitsForJointsAreDefined(copyConstructedModel);
    
    Model assignedModel;
    assignedModel = model;
    
    checkLimitsForJointsAreDefined(assignedModel);
    
    // Check the reduced model loader 
    std::vector<std::string> dofsOfModel;
    ok = dofsListFromURDF(urdfFileName,dofsOfModel);
    ASSERT_IS_TRUE(ok);

    ok = loader.loadReducedModelFromFile(urdfFileName, dofsOfModel);
    
    ASSERT_IS_TRUE(ok);
    
    Model reducedModel = loader.model();
    
    checkLimitsForJointsAreDefined(reducedModel);
}

void checkLoadReducedModelOrderIsKept(std::string urdfFileName)
{
    ModelLoader loader;
    ASSERT_IS_TRUE(loader.loadModelFromFile(urdfFileName) && loader.isValid());

    Model loadedModel = loader.model();
    //get all joints in order
    std::vector<std::string> dofsName;
    dofsName.resize(loadedModel.getNrOfDOFs());

    for (JointIndex index = 0; index < loadedModel.getNrOfJoints(); ++index) {
        IJointPtr joint = loadedModel.getJoint(index);
        std::string jointName = loadedModel.getJointName(index);

        for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof) {
            dofsName[joint->getDOFsOffset() + dof] = jointName;
        }
    }

    std::random_shuffle(dofsName.begin(), dofsName.end());

    //now load the new model and check they are the same
    ASSERT_IS_TRUE(loader.loadReducedModelFromFullModel(loadedModel, dofsName) && loader.isValid());

    Model shuffledModel = loader.model();
    for (JointIndex index = 0; index < shuffledModel.getNrOfJoints(); ++index) {
        IJointPtr joint = shuffledModel.getJoint(index);
        std::string jointName = shuffledModel.getJointName(index);
        for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof) {
            ASSERT_IS_TRUE(dofsName[joint->getDOFsOffset() + dof] == jointName);
        }
    }


}

int main()
{
    checkURDF(getAbsModelPath("/simple_model.urdf"),1,0,0,1, 1, 0, "link1");
    checkURDF(getAbsModelPath("/oneLink.urdf"),1,0,0,7,1,1,"link1");
    checkURDF(getAbsModelPath("twoLinks.urdf"),2,1,1,6,0,0,"link1");
    checkURDF(getAbsModelPath("icub_skin_frames.urdf"),39,38,32,62, 28, 28,"root_link");
    checkURDF(getAbsModelPath("iCubGenova02.urdf"),33,32,26,111, 33, 33, "root_link");
    checkURDF(getAbsModelPath("icalibrate.urdf"), 6, 5, 3, 7, 6, 6,"base");

    checkModelLoderForURDFFile(getAbsModelPath("/oneLink.urdf"));
    checkModelLoaderFromURDFString("this is not an xml", false);

    checkLimitsForJointsAreDefinedFromFileName(getAbsModelPath("iCubGenova02.urdf"));

    checkLoadReducedModelOrderIsKept(getAbsModelPath("iCubGenova02.urdf"));

    return EXIT_SUCCESS;
}
