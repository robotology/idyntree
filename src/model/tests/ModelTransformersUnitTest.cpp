// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ModelTransformers.h>

#include <iDynTree/Model.h>
#include <iDynTree/Link.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/Axis.h>
#include <iDynTree/Direction.h>
#include <iDynTree/SpatialInertia.h>

#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelTestUtils.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace iDynTree;

void checkThatOneSphereGetsAName()
{
    // Create a model with one shape with one sphere without name
    // and check that a name is correctly created
    iDynTree::Model oneSphereModel;
    iDynTree::Link oneSphereLink;
    oneSphereModel.addLink("link0", oneSphereLink);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0].resize(1);
    iDynTree::Sphere oneSphere;
    oneSphere.setLink_H_geometry(Transform::Identity());
    oneSphere.setRadius(1.0);
    oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0][0] = new iDynTree::Sphere(oneSphere);

    // Compute the model does not have a valid name
    ASSERT_IS_TRUE(!oneSphereModel.collisionSolidShapes().getLinkSolidShapes()[0][0]->isNameValid());

    // Create name with valid names
    iDynTree::Model oneSphereModelWithValidName;

    // Assert that name are valid
    ASSERT_IS_TRUE(addValidNamesToAllSolidShapes(oneSphereModel, oneSphereModelWithValidName));

    // Check if the resulting name is valid
    ASSERT_IS_TRUE(oneSphereModelWithValidName.collisionSolidShapes().getLinkSolidShapes()[0][0]->isNameValid());
    ASSERT_IS_TRUE(oneSphereModelWithValidName.collisionSolidShapes().getLinkSolidShapes()[0][0]->getName() == "link0_collision");
}

void checkRemoveAdditionalFramesFromModel()
{
    // Create random model with 10 links and 10 additional frames
    iDynTree::Model modelWithAllAdditionalFrames = getRandomModel(10, 10, SIMPLE_JOINT_TYPES | JOINT_REVOLUTE_SO2 | JOINT_SPHERICAL);

    // Create an allow list of three additional frames
    std::vector<std::string> allowedAdditionalFrames;
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 7));
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 1));
    allowedAdditionalFrames.push_back(modelWithAllAdditionalFrames.getFrameName(modelWithAllAdditionalFrames.getNrOfLinks() + 3));

    // Create a model with only the allowed additional frames
    iDynTree::Model modelWithOnlyAllowedAdditionalFrames;
    ASSERT_IS_TRUE(removeAdditionalFramesFromModel(modelWithAllAdditionalFrames, modelWithOnlyAllowedAdditionalFrames, allowedAdditionalFrames));

    // Check that the model with only the allowed additional frames has the correct number of links and additional frames
    ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.getNrOfLinks() == modelWithAllAdditionalFrames.getNrOfLinks());
    ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.getNrOfFrames() == modelWithOnlyAllowedAdditionalFrames.getNrOfLinks() + allowedAdditionalFrames.size());

    // Check that the additional frames contained in the modelWithOnlyAllowedAdditionalFrames are the one specified in modelWithOnlyAllowedAdditionalFrames
    for (size_t i = 0; i < allowedAdditionalFrames.size(); i++)
    {
        ASSERT_IS_TRUE(modelWithOnlyAllowedAdditionalFrames.isFrameNameUsed(allowedAdditionalFrames[i]));
    }
}

void testConvertSphericalJointsToThreeRevoluteJoints()
{
    // Create a simple model with one spherical joint
    iDynTree::Model originalModel;

    // Add two links
    iDynTree::Link parentLink, childLink;
    iDynTree::SpatialInertia inertia = getRandomInertia();
    parentLink.setInertia(inertia);
    childLink.setInertia(inertia);

    auto parentLinkIdx = originalModel.addLink("parent", parentLink);
    auto childLinkIdx = originalModel.addLink("child", childLink);
    originalModel.setDefaultBaseLink(parentLinkIdx);

    // Create a spherical joint between the links
    Transform joint_H_child = getRandomTransform();
    SphericalJoint sphericalJoint;
    sphericalJoint.setAttachedLinks(parentLinkIdx, childLinkIdx);
    sphericalJoint.setRestTransform(joint_H_child);
    Position jointCenter(getRandomPosition());
    sphericalJoint.setJointCenter(parentLinkIdx, jointCenter);

    originalModel.addJoint("spherical_joint", &sphericalJoint);

    // Convert spherical joint to three revolute joints
    iDynTree::Model convertedModel;
    ASSERT_IS_TRUE(convertSphericalJointsToThreeRevoluteJoints(originalModel, convertedModel,
                                                               "fake_", "rev_"));

    // Check that the converted model has the correct structure
    ASSERT_IS_TRUE(convertedModel.getNrOfLinks() == 4); // original 2 + 2 fake links
    ASSERT_IS_TRUE(convertedModel.getNrOfJoints() == 3); // 3 revolute joints

    // Check that fake links exist and have zero mass
    ASSERT_IS_TRUE(convertedModel.isLinkNameUsed("fake_spherical_joint_link1"));
    ASSERT_IS_TRUE(convertedModel.isLinkNameUsed("fake_spherical_joint_link2"));

    LinkIndex fakeLinkIdx1 = convertedModel.getLinkIndex("fake_spherical_joint_link1");
    LinkIndex fakeLinkIdx2 = convertedModel.getLinkIndex("fake_spherical_joint_link2");
    ASSERT_IS_TRUE(convertedModel.getLink(fakeLinkIdx1)->getInertia().getMass() < 1e-10);
    ASSERT_IS_TRUE(convertedModel.getLink(fakeLinkIdx2)->getInertia().getMass() < 1e-10);

    // Check that revolute joints exist
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("rev_spherical_joint_x"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("rev_spherical_joint_y"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("rev_spherical_joint_z"));

    // Check joint types are revolute
    JointIndex xJointIdx = convertedModel.getJointIndex("rev_spherical_joint_x");
    JointIndex yJointIdx = convertedModel.getJointIndex("rev_spherical_joint_y");
    JointIndex zJointIdx = convertedModel.getJointIndex("rev_spherical_joint_z");

    const IJoint* xJoint = convertedModel.getJoint(xJointIdx);
    const IJoint* yJoint = convertedModel.getJoint(yJointIdx);
    const IJoint* zJoint = convertedModel.getJoint(zJointIdx);

    ASSERT_IS_TRUE(dynamic_cast<const RevoluteJoint*>(xJoint) != nullptr);
    ASSERT_IS_TRUE(dynamic_cast<const RevoluteJoint*>(yJoint) != nullptr);
    ASSERT_IS_TRUE(dynamic_cast<const RevoluteJoint*>(zJoint) != nullptr);

    // Verify joint chain connectivity: parent -> fake1 -> fake2 -> child
    ASSERT_IS_TRUE(xJoint->getFirstAttachedLink() == convertedModel.getLinkIndex("parent"));
    ASSERT_IS_TRUE(xJoint->getSecondAttachedLink() == fakeLinkIdx1);
    ASSERT_IS_TRUE(yJoint->getFirstAttachedLink() == fakeLinkIdx1);
    ASSERT_IS_TRUE(yJoint->getSecondAttachedLink() == fakeLinkIdx2);
    ASSERT_IS_TRUE(zJoint->getFirstAttachedLink() == fakeLinkIdx2);
    ASSERT_IS_TRUE(zJoint->getSecondAttachedLink() == convertedModel.getLinkIndex("child"));

    std::cerr << "[TEST] testConvertSphericalJointsToThreeRevoluteJoints passed" << std::endl;
}

void testConvertThreeRevoluteJointsToSphericalJoint()
{
    // Create a model with three consecutive revolute joints with orthogonal axes
    iDynTree::Model originalModel;

    // Add four links (parent + 2 intermediate + child)
    iDynTree::Link parentLink, intermediate1, intermediate2, childLink;
    iDynTree::SpatialInertia parentInertia, childInertia, zeroInertia;

    parentInertia = getRandomInertia();
    childInertia = getRandomInertia();

    // Parent and child have mass, intermediate links have zero mass
    zeroInertia.zero();

    parentLink.setInertia(parentInertia);
    intermediate1.setInertia(zeroInertia);
    intermediate2.setInertia(zeroInertia);
    childLink.setInertia(childInertia);

    originalModel.addLink("parent", parentLink);
    originalModel.addLink("intermediate1", intermediate1);
    originalModel.addLink("intermediate2", intermediate2);
    originalModel.addLink("child", childLink);

    // Create three orthogonal revolute joints
    Position jointCenter(getRandomPosition());
    Transform identity = Transform::Identity();

    // X-axis rotation joint (parent -> intermediate1)
    Axis xAxis(Direction(1.0, 0.0, 0.0), jointCenter);
    RevoluteJoint xRevJoint(identity, xAxis);
    originalModel.addJoint("parent", "intermediate1", "joint_x", &xRevJoint);

    // Y-axis rotation joint (intermediate1 -> intermediate2)
    Axis yAxis(Direction(0.0, 1.0, 0.0), jointCenter);
    RevoluteJoint yRevJoint(identity, yAxis);
    originalModel.addJoint("intermediate1", "intermediate2", "joint_y", &yRevJoint);

    // Z-axis rotation joint (intermediate2 -> child)
    Axis zAxis(Direction(0.0, 0.0, 1.0), jointCenter);
    RevoluteJoint zRevJoint(identity, zAxis);
    originalModel.addJoint("intermediate2", "child", "joint_z", &zRevJoint);

    originalModel.setDefaultBaseLink(originalModel.getLinkIndex("parent"));

    // Convert three revolute joints to spherical joint
    iDynTree::Model convertedModel;
    ASSERT_IS_TRUE(convertThreeRevoluteJointsToSphericalJoint(originalModel, convertedModel,
                                                              1e-6, 1e-6, 1e-6));

    // Check that the converted model has the correct structure
    ASSERT_IS_TRUE(convertedModel.getNrOfLinks() == 2); // only parent and child remain
    ASSERT_IS_TRUE(convertedModel.getNrOfJoints() == 1); // single spherical joint

    // Check that intermediate links are removed
    ASSERT_IS_TRUE(!convertedModel.isLinkNameUsed("intermediate1"));
    ASSERT_IS_TRUE(!convertedModel.isLinkNameUsed("intermediate2"));

    // Check that parent and child links remain
    ASSERT_IS_TRUE(convertedModel.isLinkNameUsed("parent"));
    ASSERT_IS_TRUE(convertedModel.isLinkNameUsed("child"));

    // Check that old revolute joints are removed
    ASSERT_IS_TRUE(!convertedModel.isJointNameUsed("joint_x"));
    ASSERT_IS_TRUE(!convertedModel.isJointNameUsed("joint_y"));
    ASSERT_IS_TRUE(!convertedModel.isJointNameUsed("joint_z"));

    // Check that a new spherical joint exists
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint")); // Should be derived from "joint_x"

    // Verify the spherical joint type and connectivity
    JointIndex sphericalJointIdx = convertedModel.getJointIndex("joint");
    const IJoint* sphericalJoint = convertedModel.getJoint(sphericalJointIdx);
    ASSERT_IS_TRUE(dynamic_cast<const SphericalJoint*>(sphericalJoint) != nullptr);

    ASSERT_IS_TRUE(sphericalJoint->getFirstAttachedLink() == convertedModel.getLinkIndex("parent"));
    ASSERT_IS_TRUE(sphericalJoint->getSecondAttachedLink() == convertedModel.getLinkIndex("child"));

    std::cerr << "[TEST] testConvertThreeRevoluteJointsToSphericalJoint passed" << std::endl;
}

void testRoundTripSphericalJointConversion()
{
    // Test that converting spherical -> three revolute -> spherical preserves functionality
    iDynTree::Model originalModel;

    // Create a model with one spherical joint
    iDynTree::Link parentLink, childLink;
    iDynTree::SpatialInertia inertia;
    inertia = getRandomInertia();
    parentLink.setInertia(inertia);
    childLink.setInertia(inertia);

    auto parentLinkIndex = originalModel.addLink("parent", parentLink);
    auto childLinkIndex = originalModel.addLink("child", childLink);
    originalModel.setDefaultBaseLink(parentLinkIndex);

    Transform joint_H_child = getRandomTransform();
    SphericalJoint sphericalJoint;
    sphericalJoint.setAttachedLinks(parentLinkIndex, childLinkIndex);
    sphericalJoint.setRestTransform(joint_H_child);
    Position jointCenter = getRandomPosition();
    sphericalJoint.setJointCenter(parentLinkIndex, jointCenter);

    originalModel.addJoint("spherical_joint", &sphericalJoint);


    auto originalParentChildRestTransform = originalModel.getJoint(originalModel.getJointIndex("spherical_joint"))->getRestTransform(
        originalModel.getLinkIndex("child"), originalModel.getLinkIndex("parent"));

    // Convert to three revolute joints
    iDynTree::Model threeRevoluteModel;
    ASSERT_IS_TRUE(convertSphericalJointsToThreeRevoluteJoints(originalModel, threeRevoluteModel,
                                                               "fake_", "rev_"));

    // Convert back to spherical joint
    iDynTree::Model backToSphericalModel;
    ASSERT_IS_TRUE(convertThreeRevoluteJointsToSphericalJoint(threeRevoluteModel, backToSphericalModel,
                                                              1e-6, 1e-6, 1e-6));

    // Verify we're back to the original structure
    ASSERT_IS_TRUE(backToSphericalModel.getNrOfLinks() == 2);
    ASSERT_IS_TRUE(backToSphericalModel.getNrOfJoints() == 1);
    ASSERT_IS_TRUE(backToSphericalModel.isLinkNameUsed("parent"));
    ASSERT_IS_TRUE(backToSphericalModel.isLinkNameUsed("child"));
    ASSERT_IS_TRUE(backToSphericalModel.isJointNameUsed("rev_spherical_joint"));

    auto backParentChildRestTransform = backToSphericalModel.getJoint(backToSphericalModel.getJointIndex("rev_spherical_joint"))->getRestTransform(
        backToSphericalModel.getLinkIndex("child"), backToSphericalModel.getLinkIndex("parent"));


    // Check that the rest transform is preserved
    ASSERT_EQUAL_TRANSFORM(originalParentChildRestTransform, backParentChildRestTransform);
    // Check that we have a spherical joint
    JointIndex jointIdx = backToSphericalModel.getJointIndex("rev_spherical_joint"); // Name derived from first revolute joint
    const IJoint* joint = backToSphericalModel.getJoint(jointIdx);
    ASSERT_IS_TRUE(dynamic_cast<const SphericalJoint*>(joint) != nullptr);

    std::cerr << "[TEST] testRoundTripSphericalJointConversion passed" << std::endl;
}

void testSphericalJointConversionWithDisabledOptions()
{
    // Test that conversion functions work correctly when called/not called based on options
    iDynTree::Model originalModel;

    // Create a simple model with a spherical joint
    iDynTree::Link parentLink, childLink;
    iDynTree::SpatialInertia inertia;
    inertia = getRandomInertia();
    parentLink.setInertia(inertia);
    childLink.setInertia(inertia);

    auto parentLinkIdx = originalModel.addLink("parent", parentLink);
    auto childLinkIdx = originalModel.addLink("child", childLink);

    Transform joint_H_child = getRandomTransform();
    SphericalJoint sphericalJoint;
    sphericalJoint.setAttachedLinks(parentLinkIdx, childLinkIdx);
    Position jointCenter = getRandomPosition();
    sphericalJoint.setJointCenter(parentLinkIdx, jointCenter);

    originalModel.addJoint("spherical_joint", &sphericalJoint);
    originalModel.setDefaultBaseLink(parentLinkIdx);

    // Test conversion function always converts when called
    iDynTree::Model outputModel;
    ASSERT_IS_TRUE(convertSphericalJointsToThreeRevoluteJoints(originalModel, outputModel));

    // Should be converted (different from original)
    ASSERT_IS_TRUE(outputModel.getNrOfLinks() == 4); // 2 original + 2 fake
    ASSERT_IS_TRUE(outputModel.getNrOfJoints() == 3); // 3 revolute joints
    ASSERT_IS_TRUE(!outputModel.isJointNameUsed("spherical_joint"));

    std::cerr << "[TEST] testSphericalJointConversionWithDisabledOptions passed" << std::endl;
}

void testSphericalJointConversionWithNonOrthogonalAxes()
{
    // Test that conversion doesn't happen when axes are not orthogonal
    iDynTree::Model originalModel;

    // Add four links
    iDynTree::Link parentLink, intermediate1, intermediate2, childLink;
    iDynTree::SpatialInertia parentInertia, childInertia, zeroInertia;
    parentInertia = getRandomInertia();
    childInertia = getRandomInertia();
    zeroInertia.zero();

    parentLink.setInertia(parentInertia);
    intermediate1.setInertia(zeroInertia);
    intermediate2.setInertia(zeroInertia);
    childLink.setInertia(childInertia);

    originalModel.addLink("parent", parentLink);
    originalModel.addLink("intermediate1", intermediate1);
    originalModel.addLink("intermediate2", intermediate2);
    originalModel.addLink("child", childLink);

    // Create three NON-orthogonal revolute joints
    Position jointCenter = getRandomPosition();
    Transform identity = Transform::Identity();

    // Non-orthogonal axes - X, X, Z (first two are parallel, not orthogonal)
    Axis xAxis1(Direction(1.0, 0.0, 0.0), jointCenter);
    RevoluteJoint xRevJoint1(identity, xAxis1);
    originalModel.addJoint("parent", "intermediate1", "joint_x1", &xRevJoint1);

    Axis xAxis2(Direction(1.0, 0.0, 0.0), jointCenter); // Same as first axis
    RevoluteJoint xRevJoint2(identity, xAxis2);
    originalModel.addJoint("intermediate1", "intermediate2", "joint_x2", &xRevJoint2);

    Axis zAxis(Direction(0.0, 0.0, 1.0), jointCenter);
    RevoluteJoint zRevJoint(identity, zAxis);
    originalModel.addJoint("intermediate2", "child", "joint_z", &zRevJoint);

    originalModel.setDefaultBaseLink(originalModel.getLinkIndex("parent"));

    // Attempt conversion - should not detect pattern due to non-orthogonal axes
    iDynTree::Model convertedModel;
    ASSERT_IS_TRUE(convertThreeRevoluteJointsToSphericalJoint(originalModel, convertedModel,
                                                              1e-6, 1e-6, 1e-6));

    // Should be unchanged (no conversion happened)
    ASSERT_IS_TRUE(convertedModel.getNrOfLinks() == 4);
    ASSERT_IS_TRUE(convertedModel.getNrOfJoints() == 3);
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_x1"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_x2"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_z"));

    std::cerr << "[TEST] testSphericalJointConversionWithNonOrthogonalAxes passed" << std::endl;
}

void testSphericalJointConversionWithNonZeroMassIntermediateLinks()
{
    // Test that conversion doesn't happen when intermediate links have non-zero mass
    iDynTree::Model originalModel;

    // Add four links
    iDynTree::Link parentLink, intermediate1, intermediate2, childLink;
    iDynTree::SpatialInertia inertia;
    inertia = getRandomInertia();

    // All links have mass (including intermediates)
    parentLink.setInertia(inertia);
    intermediate1.setInertia(inertia); // Non-zero mass
    intermediate2.setInertia(inertia); // Non-zero mass
    childLink.setInertia(inertia);

    originalModel.addLink("parent", parentLink);
    originalModel.addLink("intermediate1", intermediate1);
    originalModel.addLink("intermediate2", intermediate2);
    originalModel.addLink("child", childLink);

    // Create three orthogonal revolute joints
    Position jointCenter = getRandomPosition();
    Transform identity = Transform::Identity();

    Axis xAxis(Direction(1.0, 0.0, 0.0), jointCenter);
    RevoluteJoint xRevJoint(identity, xAxis);
    originalModel.addJoint("parent", "intermediate1", "joint_x", &xRevJoint);

    Axis yAxis(Direction(0.0, 1.0, 0.0), jointCenter);
    RevoluteJoint yRevJoint(identity, yAxis);
    originalModel.addJoint("intermediate1", "intermediate2", "joint_y", &yRevJoint);

    Axis zAxis(Direction(0.0, 0.0, 1.0), jointCenter);
    RevoluteJoint zRevJoint(identity, zAxis);
    originalModel.addJoint("intermediate2", "child", "joint_z", &zRevJoint);

    originalModel.setDefaultBaseLink(originalModel.getLinkIndex("parent"));

    // Attempt conversion - should not detect pattern due to non-zero mass intermediate links
    iDynTree::Model convertedModel;
    ASSERT_IS_TRUE(convertThreeRevoluteJointsToSphericalJoint(originalModel, convertedModel,
                                                              1e-6, 1e-6, 1e-6));

    // Should be unchanged (no conversion happened)
    ASSERT_IS_TRUE(convertedModel.getNrOfLinks() == 4);
    ASSERT_IS_TRUE(convertedModel.getNrOfJoints() == 3);
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_x"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_y"));
    ASSERT_IS_TRUE(convertedModel.isJointNameUsed("joint_z"));

    std::cerr << "[TEST] testSphericalJointConversionWithNonZeroMassIntermediateLinks passed" << std::endl;
}


int main()
{
    checkThatOneSphereGetsAName();
    checkRemoveAdditionalFramesFromModel();

    // Add the new spherical joint conversion tests
    testConvertSphericalJointsToThreeRevoluteJoints();
    testConvertThreeRevoluteJointsToSphericalJoint();
    testRoundTripSphericalJointConversion();
    testSphericalJointConversionWithDisabledOptions();
    testSphericalJointConversionWithNonOrthogonalAxes();
    testSphericalJointConversionWithNonZeroMassIntermediateLinks();

    return EXIT_SUCCESS;
}
