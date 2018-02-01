/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/DenavitHartenberg.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <iDynTree/KinDynComputations.h>

#include "testModels.h"

using namespace iDynTree;

bool ExtractReducedJointPosFromFullModel(const Model& fullModel,
                                         const VectorDynSize& fullJntPos,
                                         const Model& reducedModel,
                                         VectorDynSize& reducedJntPos)
{
    assert(fullJntPos.size() == fullModel.getNrOfPosCoords());
    reducedJntPos.resize(reducedModel.getNrOfJoints());

    for (JointIndex reducedJntIdx=0; reducedJntIdx < reducedModel.getNrOfJoints(); reducedJntIdx++)
    {
        // Check that joint is present in the full model
        FrameIndex fullJntIdx = fullModel.getJointIndex(reducedModel.getJointName(reducedJntIdx));
        assert(fullJntIdx != FRAME_INVALID_INDEX);

        // Compute the dofs of the joints and full/reduced model offsets
        assert(fullModel.getJoint(fullJntIdx)->getNrOfDOFs() ==
               reducedModel.getJoint(reducedJntIdx)->getNrOfDOFs());
        unsigned int nrOfDofs =fullModel.getJoint(fullJntIdx)->getNrOfDOFs();
        size_t fullDofOffset = fullModel.getJoint(fullJntIdx)->getDOFsOffset();
        size_t reducedDofOffset = reducedModel.getJoint(reducedJntIdx)->getDOFsOffset();

        // Copy the jnt pos
        for (int j=0; j < nrOfDofs; j++)
        {
            reducedJntPos(reducedDofOffset+j) = fullJntPos(fullDofOffset+j);
        }

    }

    return true;
}

void checkDHConversionsOnArbitraryChains(const Model& model)
{
    KinDynComputations kinDynComp;

    bool modelOk = kinDynComp.loadRobotModel(model);
    ASSERT_IS_TRUE(modelOk);

    // Set a random joint state for the model, that will be used
    // to check the transform provided by the DH chain
    VectorDynSize fullJntPos, reducedJntPos;
    getRandomJointPositions(fullJntPos, model);
    kinDynComp.setJointPos(fullJntPos);

    for (int i=0; i < 10; i++)
    {
        // Get a random couple of frames
        FrameIndex baseFrameIdx   = getRandomInteger(0, model.getNrOfFrames()-1);
        FrameIndex distalFrameIdx = getRandomInteger(0, model.getNrOfFrames()-1);

        std::string baseFrame = model.getFrameName(baseFrameIdx);
        std::string distalFrame = model.getFrameName(distalFrameIdx);

        std::cerr << "Computing chain with base " << baseFrame << " and distal " << distalFrame << std::endl;

        // Extract the DH parameters for the chain connecting this two frames
        DHChain extractedDHParams;
        bool ok = ExtractDHChainFromModel(model, baseFrame, distalFrame, extractedDHParams);
        ASSERT_IS_TRUE(ok);

        // Create a model from a DH chain
        Model extractedChain;
        ok = CreateModelFromDHChain(extractedDHParams,
                                    extractedChain);
        ASSERT_IS_TRUE(ok);

        // Extract the state of the Chain from the complete state
        ExtractReducedJointPosFromFullModel(model, fullJntPos, extractedChain, reducedJntPos);
        KinDynComputations chainKinDynComp;
        ok = chainKinDynComp.loadRobotModel(extractedChain);
        ASSERT_IS_TRUE(ok);
        chainKinDynComp.setJointPos(reducedJntPos);

        // Compare the two frames
        ASSERT_EQUAL_TRANSFORM_TOL(kinDynComp.getRelativeTransform(baseFrame, distalFrame),
                                   chainKinDynComp.getRelativeTransform("baseFrame", "distalFrame"), 1e-5);
    }
}

// Check DH idempotemcy.
// Note that in general it is possible
// that this idempotemcy is not guaranteeed
// (i.e. two different DHChain generate the same model)
// So pay attention to run this check on DHChain for which
// you know that idempotemcy holds
void checkDHIdempotency(const DHChain& chain, bool checkIdempotency=true)
{
    // DH --> Model
    Model chainModel;
    bool ok = chain.toModel(chainModel);
    ASSERT_IS_TRUE(ok);

    // Model --> DH
    DHChain chainCheck;
    ok = chainCheck.fromModel(chainModel, "baseFrame", "distalFrame");
    ASSERT_IS_TRUE(ok);

    // Check DHChain for idempotemcy
    if (checkIdempotency)
    {
        ASSERT_EQUAL_TRANSFORM(chain.getH0(), chainCheck.getH0());
        for (int i=0; i < chain.getNrOfDOFs(); i++)
        {
            DHLink dhLink = chain(i);
            DHLink dhLinkCheck = chainCheck(i);

            ASSERT_EQUAL_DOUBLE(dhLink.A, dhLinkCheck.A);
            ASSERT_EQUAL_DOUBLE(dhLink.D, dhLinkCheck.D);
            ASSERT_EQUAL_DOUBLE(dhLink.Alpha, dhLinkCheck.Alpha);
            ASSERT_EQUAL_DOUBLE(dhLink.Offset, dhLinkCheck.Offset);
        }
        ASSERT_EQUAL_TRANSFORM(chain.getHN(), chainCheck.getHN());
    }

    checkDHConversionsOnArbitraryChains(chainModel);
}

int main()
{
    // Check simple DH chains

    // Simple 1 dof
    DHChain dh1dof;
    dh1dof.setH0(Transform::Identity());
    dh1dof.setHN(Transform::Identity());
    dh1dof.setNrOfDOFs(1);
    dh1dof.setDOFName(0, "joint0");
    dh1dof(0).A = dh1dof(0).D = dh1dof(0).Alpha = dh1dof(0).Offset = 0.0;
    checkDHIdempotency(dh1dof);

    dh1dof(0).A = 1.0;
    checkDHIdempotency(dh1dof);
    dh1dof(0).A = 0.0;

    dh1dof(0).D = 1.0;
    checkDHIdempotency(dh1dof);
    dh1dof(0).D = 0.0;

    dh1dof(0).Alpha = 1.0;
    checkDHIdempotency(dh1dof);
    dh1dof(0).Alpha = 0.0;

    dh1dof(0).Offset = 1.0;
    // An offset in the last link cannot be distinguished from a rotation due
    // to HN, not checking idempotemcy in this case
    bool checkIdempotency=false;
    checkDHIdempotency(dh1dof, checkIdempotency);
    dh1dof(0).Offset = 0.0;

    // Simple 2 dofs
    DHChain dh2dof;
    dh2dof.setH0(Transform::Identity());
    dh2dof.setHN(Transform::Identity());
    dh2dof.setNrOfDOFs(2);
    dh2dof.setDOFName(0, "joint0");
    dh2dof.setDOFName(1, "joint1");

    dh2dof(0).A = dh2dof(0).D = dh2dof(0).Alpha = dh2dof(0).Offset = 0.0;
    dh2dof(1).A = dh2dof(1).D = dh2dof(1).Alpha = dh2dof(1).Offset = 0.0;
    //checkDHIdempotency(dh2dof);

    dh2dof(0).A = dh2dof(1).A = 1.0;
    checkDHIdempotency(dh2dof);
    dh2dof(0).A = dh2dof(1).A = 0.0;

    dh2dof(0).D = dh2dof(1).D = 1.0;
    checkDHIdempotency(dh2dof);
    dh2dof(0).D = dh2dof(1).D  = 0.0;

    dh2dof(0).Alpha = dh2dof(1).Alpha  = 1.0;
    checkDHIdempotency(dh2dof);
    dh2dof(0).Alpha = dh2dof(1).Alpha  = 0.0;

    dh2dof(0).Offset = dh2dof(1).Offset  = 1.0;
    // An offset in the last link cannot be distinguished from a rotation due
    // to HN, not checking idempotemcy in this case
    checkIdempotency=false;
    checkDHIdempotency(dh2dof, checkIdempotency);
    dh2dof(0).Offset = dh2dof(1).Offset  = 0.0;


    for (unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "---> Checking DH conversions test on " << urdfFileName << std::endl;
        ModelLoader modelLoader;
        bool ok = modelLoader.loadModelFromFile(urdfFileName);
        ASSERT_IS_TRUE(ok);
        checkDHConversionsOnArbitraryChains(modelLoader.model());
    }
}
