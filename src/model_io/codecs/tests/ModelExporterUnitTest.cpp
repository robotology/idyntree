// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

#include <iDynTree/TestUtils.h>


#include <iDynTree/Model.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/ModelExporter.h>


#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <memory>
#include <fstream>

using namespace iDynTree;

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

void checkSolidAreEqual(SolidShape* solid, SolidShape* solidCheck)
{
    ASSERT_IS_TRUE(solid->isBox() == solidCheck->isBox());
    ASSERT_IS_TRUE(solid->isCylinder() == solidCheck->isCylinder());
    ASSERT_IS_TRUE(solid->isSphere() == solidCheck->isSphere());
    ASSERT_IS_TRUE(solid->isExternalMesh() == solidCheck->isExternalMesh());

    ASSERT_IS_TRUE(solid->getName() == solidCheck->getName());
    ASSERT_IS_TRUE(solid->isNameValid() == solidCheck->isNameValid());

    ASSERT_EQUAL_TRANSFORM(solid->getLink_H_geometry(), solidCheck->getLink_H_geometry());

    if (solid->isBox()) {
        const Box* box = solid->asBox();
        const Box* boxCheck = solidCheck->asBox();
        ASSERT_EQUAL_DOUBLE(box->getX(), boxCheck->getX());
        ASSERT_EQUAL_DOUBLE(box->getY(), boxCheck->getY());
        ASSERT_EQUAL_DOUBLE(box->getZ(), boxCheck->getZ());

    } else if (solid->isCylinder()) {
        const Cylinder* cylinder = solid->asCylinder();
        const Cylinder* cylinderCheck = solidCheck->asCylinder();
        ASSERT_EQUAL_DOUBLE(cylinder->getRadius(), cylinderCheck->getRadius());
        ASSERT_EQUAL_DOUBLE(cylinder->getLength(), cylinderCheck->getLength());

    } else if (solid->isSphere()) {
        const Sphere* sphere = solid->asSphere();
        const Sphere* sphereCheck = solidCheck->asSphere();
        ASSERT_EQUAL_DOUBLE(sphere->getRadius(), sphereCheck->getRadius());

    } else if (solid->isExternalMesh()) {
        const ExternalMesh* mesh = solid->asExternalMesh();
        const ExternalMesh* meshCheck = solidCheck->asExternalMesh();
        ASSERT_EQUAL_VECTOR(mesh->getScale(), meshCheck->getScale());
        ASSERT_IS_TRUE(mesh->getFilename() == meshCheck->getFilename());
    } else {
        ASSERT_IS_TRUE(false);
    }

}


void checkImportExportURDF(std::string fileName)
{
    // Import, export and re-import a URDF file, and check that the key properties of the model are mantained
    ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(fileName, "urdf");
    Model model = mdlLoader.model();
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
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
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(model), getNrOfVisuals(modelReloaded));
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(model), getNrOfCollisions(modelReloaded));

    // Verify that the link correspond (note that the serialization could have changed)
    for(int lnkIndex=0; lnkIndex < model.getNrOfLinks(); lnkIndex++) {
        LinkIndex lnkIndexInReloaded = modelReloaded.getLinkIndex(model.getLinkName(lnkIndex));
        ASSERT_IS_TRUE(lnkIndexInReloaded != LINK_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getLinkName(lnkIndex) == modelReloaded.getLinkName(lnkIndexInReloaded));
        SpatialInertia inertia = model.getLink(lnkIndex)->getInertia();
        SpatialInertia inertiaReloaded = modelReloaded.getLink(lnkIndexInReloaded)->getInertia();
        std::cerr << "Testing inertia of link " << model.getLinkName(lnkIndex) << std::endl;
        ASSERT_EQUAL_MATRIX(inertia.asMatrix(), inertiaReloaded.asMatrix());

        // For each link, verify that the visual and collision shape correspond

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.visualSolidShapes().getLinkSolidShapes()[lnkIndex].size(),
                            modelReloaded.visualSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded].size());

        // Then, if there is only one element, verify that it matches
        if (model.visualSolidShapes().getLinkSolidShapes()[lnkIndex].size() == 1) {
            SolidShape* solidInModel = model.visualSolidShapes().getLinkSolidShapes()[lnkIndex][0];
            SolidShape* solidInModelReloaded = modelReloaded.visualSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded][0];
            std::cerr << "original: " << solidInModel->getLink_H_geometry().toString() << std::endl
                      << "reloaded: " << solidInModelReloaded->getLink_H_geometry().toString() << std::endl;
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex].size(),
                            modelReloaded.collisionSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded].size());

        // Then, if there is only one element, verify that it matches
        if (model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex].size() == 1) {
            SolidShape* solidInModel = model.collisionSolidShapes().getLinkSolidShapes()[lnkIndex][0];
            SolidShape* solidInModelReloaded = modelReloaded.collisionSolidShapes().getLinkSolidShapes()[lnkIndexInReloaded][0];
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }
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

void testFramesNotInTraversal() {
    // Create a model with two different roots and export only one.
    Model model;
    SpatialInertia zeroInertia;
    zeroInertia.zero();

    Link link;
    link.setInertia(zeroInertia);

    // Create a 2 links - one joint model.
    model.addLink("link_1_1", link);
    model.addLink("link_1_2", link);

    auto joint = std::make_unique<RevoluteJoint>();
    model.addJoint("link_1_1", "link_1_2", "j1", joint.get());

    // Add a frame to the body.
    model.addAdditionalFrameToLink("link_1_2", "f1", Transform::Identity());

    // Add also another link not connected to the other body.
    model.addLink("link_2_1", link);

    // Add a frame to this body too.
    model.addAdditionalFrameToLink("link_2_1", "f2", Transform::Identity());

    // Export the model. We want to export only the first body.
    ModelExporter exporter;
    // Set the root link to link_1_1.
    ModelExporterOptions options;
    options.baseLink = "link_1_1";
    exporter.setExportingOptions(options);

    bool ok = exporter.init(model, SensorsList());
    ASSERT_IS_TRUE(ok);
    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    // Reload the model.
    ModelLoader loader;

    ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model modelReloaded = loader.model();

    // Check that the reloaded model contains only the 2 links - 1 joint model.
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfLinks(), 2);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfJoints(), 1);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfDOFs(), 1);
    ASSERT_EQUAL_DOUBLE(modelReloaded.getNrOfFrames(), 2 + 1);
}

void testJointAxisWithNonZeroOriginButPassingThroughChildLinkFrameOrigin() {
    // iDynTree has no constraint of where the joint axis passes w.r.t. to
    // the link frames of the link it connects, while URDF constraints the
    // joint axis to pass through the origin of the child link frame

    // This test checks that a model with a non-zero offset origin but that
    // passes through the child link origin is correctly exported
    Model model;
    SpatialInertia zeroInertia;
    zeroInertia.zero();

    Link link;
    link.setInertia(zeroInertia);

    // Create a 2 links - one joint model.
    std::string link1Name = "link_1";
    std::string link2Name = "link_2";
    std::string jointName = "j1";

    model.addLink(link1Name, link);
    model.addLink(link2Name, link);

    auto joint = std::make_unique<RevoluteJoint>();

    // Add a offset in the x direction between link_1 and link_2
    iDynTree::Transform link1_X_link2 = iDynTree::Transform::Identity();
    link1_X_link2.setPosition(iDynTree::Position(1.0, 0.0, 0.0));
    joint->setRestTransform(link1_X_link2);

    // Mark the rotation of the joint along z, and add a offset of the axis along z
    iDynTree::Axis axis;
    axis.setDirection(iDynTree::Direction(0.0, 0.0, 1.0));
    axis.setOrigin(iDynTree::Position(0.0, 0.0, 1.0));
    joint->setAxis(axis, model.getLinkIndex(link2Name));

    model.addJoint(link1Name, link2Name, jointName, joint.get());

    ModelExporter exporter;
    bool ok = exporter.init(model);
    ASSERT_IS_TRUE(ok);
    std::string urdfString;
    ok = exporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);
    // Reload the model.
    ModelLoader loader;

    ok = loader.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);

    // Reload model
    Model modelReloaded = loader.model();

    // Check that the reloaded have the same transform of the original model
    iDynTree::VectorDynSize jointPos;
    jointPos.resize(model.getNrOfPosCoords());
    for (size_t i=0; i++; i<10)
    {
        // Arbitrary joint angles
        jointPos(0) = 0.1*i-0.5;
        iDynTree::Transform link1_H_link2_orig =
            model.getJoint(model.getJointIndex(jointName))->getTransform(jointPos, model.getLinkIndex(link1Name), model.getLinkIndex(link2Name));
        iDynTree::Transform link1_H_link2_reloaded =
            modelReloaded.getJoint(modelReloaded.getJointIndex(jointName))->getTransform(jointPos, modelReloaded.getLinkIndex(link1Name), modelReloaded.getLinkIndex(link2Name));

        ASSERT_EQUAL_TRANSFORM(link1_H_link2_orig, link1_H_link2_reloaded);
    }

    // Create a new model with an axis that does not pass through the origin of the child link and verify that it can't be loaded
    Model modelThatCantBeExportedToUrdf;
    modelThatCantBeExportedToUrdf.addLink(link1Name, link);
    modelThatCantBeExportedToUrdf.addLink(link2Name, link);
    auto jointNew = std::make_unique<RevoluteJoint>();

    // Add a offset in the x direction between link_1 and link_2
    iDynTree::Transform link1_X_link2New = iDynTree::Transform::Identity();
    link1_X_link2New.setPosition(iDynTree::Position(1.0, 0.0, 0.0));
    jointNew->setRestTransform(link1_X_link2);

    // Mark the rotation of the joint along z, and add a offset of the axis along y
    iDynTree::Axis axisNew;
    axisNew.setDirection(iDynTree::Direction(0.0, 0.0, 1.0));
    axisNew.setOrigin(iDynTree::Position(0.0, 1.0, 0.0));
    jointNew->setAxis(axisNew, modelThatCantBeExportedToUrdf.getLinkIndex(link2Name));

    modelThatCantBeExportedToUrdf.addJoint(link1Name, link2Name, jointName, jointNew.get());

    ModelExporter exporterNew;
    ok = exporterNew.init(modelThatCantBeExportedToUrdf);
    ASSERT_IS_TRUE(ok);
    ok = exporterNew.exportModelToString(urdfString);
    ASSERT_IS_FALSE(ok);
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
        std::cout << "Checking model import/export test on " << urdfFileName << std::endl;
        checkImportExportURDF(urdfFileName);
    }

    testFramesNotInTraversal();
    testJointAxisWithNonZeroOriginButPassingThroughChildLinkFrameOrigin();


    return EXIT_SUCCESS;
}
