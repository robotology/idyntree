// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SimpleLeggedOdometry.h>

#include "testModels.h"

#include <iDynTree/TestUtils.h>
#include <iDynTree/ModelLoader.h>


using namespace iDynTree;

std::vector<std::string> getCanonical_iCubJoints()
{
    std::vector<std::string> consideredJoints;

    consideredJoints.push_back("torso_pitch");
    consideredJoints.push_back("torso_roll");
    consideredJoints.push_back("torso_yaw");
    consideredJoints.push_back("neck_pitch");
    consideredJoints.push_back("neck_roll");
    consideredJoints.push_back("neck_yaw");
    consideredJoints.push_back("l_shoulder_pitch");
    consideredJoints.push_back("l_shoulder_roll");
    consideredJoints.push_back("l_shoulder_yaw");
    consideredJoints.push_back("l_elbow");
    consideredJoints.push_back("r_shoulder_pitch");
    consideredJoints.push_back("r_shoulder_roll");
    consideredJoints.push_back("r_shoulder_yaw");
    consideredJoints.push_back("r_elbow");
    consideredJoints.push_back("l_hip_pitch");
    consideredJoints.push_back("l_hip_roll");
    consideredJoints.push_back("l_hip_yaw");
    consideredJoints.push_back("l_knee");
    consideredJoints.push_back("l_ankle_pitch");
    consideredJoints.push_back("l_ankle_roll");
    consideredJoints.push_back("r_hip_pitch");
    consideredJoints.push_back("r_hip_roll");
    consideredJoints.push_back("r_hip_yaw");
    consideredJoints.push_back("r_knee");
    consideredJoints.push_back("r_ankle_pitch");
    consideredJoints.push_back("r_ankle_roll");

    return consideredJoints;
}

void setDOFsSubSetPositionsInDegrees(const iDynTree::Model & model,
                                     const std::vector<std::string> & dofNames,
                                     const std::vector<double> & dofPositionsInDegrees,
                                     JointPosDoubleArray & qj)
{
    for(int i=0; i < dofNames.size(); i++)
    {
        iDynTree::JointIndex jntIdx = model.getJointIndex(dofNames[i]);
        size_t dofOffset = model.getJoint(jntIdx)->getDOFsOffset();
        qj(dofOffset) = deg2rad(dofPositionsInDegrees[i]);
    }
}


int main()
{
    SimpleLeggedOdometry simpleOdometry;

    std::vector<std::string> consideredJoints = getCanonical_iCubJoints();

    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadReducedModelFromFile(getAbsModelPath("iCubDarmstadt01.urdf"),consideredJoints);
    ok =  ok && simpleOdometry.setModel(mdlLoader.model());

    ASSERT_IS_TRUE(ok);

    JointPosDoubleArray qj(simpleOdometry.model());

    qj.zero();

    // Initialize the odometry
    Transform l_sole_H_world = iDynTree::getRandomTransform();

    ok = simpleOdometry.init("l_sole",l_sole_H_world);

    ok = simpleOdometry.updateKinematics(qj);

    ASSERT_IS_TRUE(ok);

    Transform world_H_rootLink1 = simpleOdometry.getWorldLinkTransform(simpleOdometry.model().getLinkIndex("root_link"));

    ASSERT_IS_TRUE(simpleOdometry.changeFixedFrame("r_sole"));

    Transform world_H_rootLink2 = simpleOdometry.getWorldLinkTransform(simpleOdometry.model().getLinkIndex("root_link"));

    ASSERT_IS_TRUE(simpleOdometry.changeFixedFrame("head"));

    Transform world_H_rootLink3 = simpleOdometry.getWorldLinkTransform(simpleOdometry.model().getLinkIndex("root_link"));

    ASSERT_EQUAL_TRANSFORM(world_H_rootLink1,world_H_rootLink2);
    ASSERT_EQUAL_TRANSFORM(world_H_rootLink1,world_H_rootLink3);

    // Set a desired configuration
    std::vector<std::string> dofNames;
    std::vector<double> dofPositionsInDegrees;


    return EXIT_SUCCESS;
}
