// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelTestUtils.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/TestUtils.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// For modelTransformers testing
#include <iDynTree/ModelTransformers.h>
#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/Dynamics.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/SubModel.h>

using namespace iDynTree;


void createCopyAndDestroy(const Model & model)
{
    Model * p_model = new Model(model);
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfFrames(),model.getNrOfFrames());
    *p_model = model;
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfFrames(),model.getNrOfFrames());
    delete p_model;
}

void checkComputeTraversal(const Model & model)
{
    Traversal traversal;
    bool ok = model.computeFullTreeTraversal(traversal);

    ASSERT_EQUAL_DOUBLE(ok,true);
    ASSERT_EQUAL_DOUBLE(traversal.getNrOfVisitedLinks(),model.getNrOfLinks());
}

void checkNeighborSanity(const Model & model, bool verbose)
{
    for(size_t link =0; link < model.getNrOfLinks(); link++ )
    {
        if( verbose )
        {
            std::cout << "Link " << model.getLinkName(link) << " has "
                      << model.getNrOfNeighbors(link) << " neeighbors" << std::endl;
        }

        for(unsigned int neigh_i = 0; neigh_i < model.getNrOfNeighbors(link); neigh_i++ )
        {
            LinkIndex neighIndex = model.getNeighbor(link,neigh_i).neighborLink;
            std::string neighName = model.getLinkName(neighIndex);
            if( verbose )
            {
                std::cout << "neighbor " << neigh_i << " is " << neighName << std::endl;
            }
        }
    }
}

bool isStringInVector(const std::string & str,
                      const std::vector<std::string> & vec)
{
    return std::find(vec.begin(), vec.end(), str) != vec.end();
}

void getRandomSubsetOfJoints(const Model & model,
                             size_t nrOfJointsInSubset,
                             std::vector<std::string>& subsetOfJoints)
{
    while( subsetOfJoints.size() < nrOfJointsInSubset )
    {
        JointIndex randomJoint = (JointIndex) getRandomInteger(0,model.getNrOfJoints()-1);
        std::string randomJointName = model.getJointName(randomJoint);

        // If the random added joint is not in the vector, add it
        if( !isStringInVector(randomJointName,subsetOfJoints) )
        {
            subsetOfJoints.push_back(randomJointName);
        }
    }
}

void getRandomJointPositonsForJointsNotInReducedModels(const Model & fullModel,
                                                       const std::vector<std::string>& subsetOfJointsInReducedModel,
                                                       std::unordered_map<std::string, std::vector<double>>& removedJointPositions,
                                                       FreeFloatingPos& fullModelPos)
{
    for(JointIndex jntIndex = 0; jntIndex < fullModel.getNrOfJoints(); jntIndex++)
    {
        // Check if joint is in reduced model
        std::string jointName = fullModel.getJointName(jntIndex);

        if (!isStringInVector(jointName, subsetOfJointsInReducedModel))
        {
            size_t nrOfPosCoords = fullModel.getJoint(jntIndex)->getNrOfPosCoords();

            // Generate random values for all position coordinates
            std::vector<double> jointConfigs(nrOfPosCoords);
            for (size_t i = 0; i < nrOfPosCoords; i++)
            {
                jointConfigs[i] = iDynTree::getRandomDouble();
                fullModelPos.jointPos()(fullModel.getJoint(jntIndex)->getPosCoordsOffset() + i) = jointConfigs[i];
            }

            // Use normalizeJointPosCoords to handle any required normalization (e.g., quaternions for spherical joints)
            size_t jointOffset = fullModel.getJoint(jntIndex)->getPosCoordsOffset();
            iDynTree::Span<double> jointSpan(fullModelPos.jointPos().data() + jointOffset, nrOfPosCoords);
            fullModel.getJoint(jntIndex)->normalizeJointPosCoords(jointSpan);

            // Update the stored values with the normalized coordinates
            for (size_t i = 0; i < nrOfPosCoords; i++)
            {
                jointConfigs[i] = fullModelPos.jointPos()(fullModel.getJoint(jntIndex)->getPosCoordsOffset() + i);
            }
            removedJointPositions[jointName] = jointConfigs;
        }
    }
}

class RNEAHelperClass
{
private:
    Model model;
    LinkVelArray linkVels;
    LinkAccArray linkAccs;
    LinkNetExternalWrenches linkExtF;
    LinkInternalWrenches linkIntF;
    Traversal traversal;

public:
    RNEAHelperClass(const Model& _model): model(_model),
                                          linkVels(_model),
                                          linkAccs(_model),
                                          linkExtF(_model),
                                          linkIntF(_model)
    {
        model.computeFullTreeTraversal(traversal);
    }

    bool runRNEA(const FreeFloatingPos & pos, const FreeFloatingVel & vel,
                 const FreeFloatingAcc & acc, FreeFloatingGeneralizedTorques & trqs)
    {
        bool ok = true;
        ok = ok && ForwardVelAccKinematics(model,traversal,
                                           pos,vel,acc,
                                           linkVels,linkAccs);
        ok = ok && RNEADynamicPhase(model,traversal,
                                    pos.jointPos(),linkVels,linkAccs,
                                    linkExtF,linkIntF,trqs);

        return ok;
    }
};

/**
 * Copy a vector of the position coordinates from the reduced model to a full model.
 *
 */
template<typename vectorType>
void copyFromReducedToFull(const vectorType & reducedVector,
                            vectorType & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            assert(jntInFullModel->getNrOfDOFs() > 0);
            assert(fullVector.size() == fullModel.getNrOfPosCoords());
            assert(reducedVector.size() == reducedModel.getNrOfPosCoords());
            assert(jntInFullModel->getPosCoordsOffset() < fullVector.size());
            assert(jnt->getPosCoordsOffset() < reducedVector.size());

            // Copy all position coordinates for this joint
            for(int i = 0; i < jnt->getNrOfPosCoords(); i++)
            {
                assert(jntInFullModel->getPosCoordsOffset() + i < fullVector.size());
                assert(jnt->getPosCoordsOffset() + i < reducedVector.size());
                fullVector(jntInFullModel->getPosCoordsOffset() + i) = reducedVector(jnt->getPosCoordsOffset() + i);
            }
        }
    }
}


/**
 * Copy a vector of the position coordinates from the full model to a reduced model.
 *
 */
template<typename vectorType>
void copyFromFullToReduced(      vectorType & reducedVector,
                      const vectorType & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            // Copy all position coordinates for this joint
            for(int i = 0; i < jnt->getNrOfPosCoords(); i++)
            {
                assert(jnt->getPosCoordsOffset() + i < reducedVector.size());
                assert(jntInFullModel->getPosCoordsOffset() + i < fullVector.size());
                reducedVector(jnt->getPosCoordsOffset() + i) = fullVector(jntInFullModel->getPosCoordsOffset() + i);
            }
        }
    }
}

/**
 * Specialized version of copyFromFullToReduced for JointPosDoubleArray (position coordinates)
 */
void copyFromFullToReduced(      JointPosDoubleArray & reducedVector,
                      const JointPosDoubleArray & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            // Copy all position coordinates for this joint
            for(int i = 0; i < jnt->getNrOfPosCoords(); i++)
            {
                assert(jnt->getPosCoordsOffset() + i < reducedVector.size());
                assert(jntInFullModel->getPosCoordsOffset() + i < fullVector.size());
                reducedVector(jnt->getPosCoordsOffset() + i) = fullVector(jntInFullModel->getPosCoordsOffset() + i);
            }
        }
    }
}

/**
 * Specialized version of copyFromFullToReduced for JointDOFsDoubleArray (velocity/acceleration/torque)
 */
void copyFromFullToReduced(      JointDOFsDoubleArray & reducedVector,
                      const JointDOFsDoubleArray & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            // Copy all DOF values for this joint
            for(int i = 0; i < jnt->getNrOfDOFs(); i++)
            {
                assert(jnt->getDOFsOffset() + i < reducedVector.size());
                assert(jntInFullModel->getDOFsOffset() + i < fullVector.size());
                reducedVector(jnt->getDOFsOffset() + i) = fullVector(jntInFullModel->getDOFsOffset() + i);
            }
        }
    }
}

/**
 * Specialized version of copyFromReducedToFull for JointPosDoubleArray (position coordinates)
 */
void copyFromReducedToFull(const JointPosDoubleArray & reducedVector,
                            JointPosDoubleArray & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            assert(jntInFullModel->getNrOfDOFs() > 0);
            assert(fullVector.size() == fullModel.getNrOfPosCoords());
            assert(reducedVector.size() == reducedModel.getNrOfPosCoords());
            assert(jntInFullModel->getPosCoordsOffset() < fullVector.size());
            assert(jnt->getPosCoordsOffset() < reducedVector.size());

            // Copy all position coordinates for this joint
            for(int i = 0; i < jnt->getNrOfPosCoords(); i++)
            {
                assert(jntInFullModel->getPosCoordsOffset() + i < fullVector.size());
                assert(jnt->getPosCoordsOffset() + i < reducedVector.size());
                fullVector(jntInFullModel->getPosCoordsOffset() + i) = reducedVector(jnt->getPosCoordsOffset() + i);
            }

            // Use normalizeJointPosCoords to handle any required normalization (e.g., quaternions for spherical joints)
            size_t jointOffset = jntInFullModel->getPosCoordsOffset();
            iDynTree::Span<double> jointSpan(fullVector.data() + jointOffset, jnt->getNrOfPosCoords());
            jntInFullModel->normalizeJointPosCoords(jointSpan);
        }
    }
}

/**
 * Specialized version of copyFromReducedToFull for JointDOFsDoubleArray (velocity/acceleration/torque)
 */
void copyFromReducedToFull(const JointDOFsDoubleArray & reducedVector,
                            JointDOFsDoubleArray & fullVector,
                      const Model & reducedModel,
                      const Model & fullModel)
{
    for(JointIndex jntReduced = 0; jntReduced < reducedModel.getNrOfJoints(); jntReduced++ )
    {
        IJointConstPtr jnt = reducedModel.getJoint(jntReduced);
        std::string jointName = reducedModel.getJointName(jntReduced);

        if( jnt->getNrOfDOFs() > 0 )
        {
            IJointConstPtr jntInFullModel = fullModel.getJoint(fullModel.getJointIndex(jointName));

            assert(jntInFullModel->getNrOfDOFs() > 0);
            assert(fullVector.size() == fullModel.getNrOfDOFs());
            assert(reducedVector.size() == reducedModel.getNrOfDOFs());
            assert(jntInFullModel->getDOFsOffset() < fullVector.size());
            assert(jnt->getDOFsOffset() < reducedVector.size());

            // Copy all DOF values for this joint
            for(int i = 0; i < jnt->getNrOfDOFs(); i++)
            {
                assert(jntInFullModel->getDOFsOffset() + i < fullVector.size());
                assert(jnt->getDOFsOffset() + i < reducedVector.size());
                fullVector(jntInFullModel->getDOFsOffset() + i) = reducedVector(jnt->getDOFsOffset() + i);
            }
        }
    }
}

/**
 * \todo Move outsite in proper ModelTransformersUnitTest file.
 */
void checkReducedModel(const Model & model)
{
    // Create a random reduced version of the model
    // and check that the RNEA for the full model
    // and for the reduced model (with the removed
    // joint positions, velocity and accelerations set to 0)
    // is giving the same results
    for(size_t jnts=0; jnts < model.getNrOfJoints(); jnts += 5)
    {
        FreeFloatingPos fullPos(model);

        // Initialize the joint positions with random values first
        getRandomJointPositions(fullPos.jointPos(), model);

        std::vector<std::string> jointInReducedModel;
        getRandomSubsetOfJoints(model,jnts,jointInReducedModel);

        // Get random positions for reduced models
        std::unordered_map<std::string, std::vector<double>> removedJointPositions;
        getRandomJointPositonsForJointsNotInReducedModels(model, jointInReducedModel, removedJointPositions, fullPos);

        Model reducedModel;
        bool ok = createReducedModel(model, jointInReducedModel, reducedModel, removedJointPositions);

        ASSERT_EQUAL_DOUBLE(ok,1.0);

        // Check that the two models have the same number of frames
        ASSERT_EQUAL_DOUBLE(reducedModel.getNrOfJoints(),jnts);
        ASSERT_EQUAL_DOUBLE(reducedModel.getNrOfJoints(),jointInReducedModel.size());
        ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(),reducedModel.getNrOfFrames());

        // Run RNEA
        RNEAHelperClass fullRNEA(model);
        RNEAHelperClass reducedRNEA(reducedModel);

        FreeFloatingPos reducedPos(reducedModel);
        FreeFloatingVel reducedVel(reducedModel);
        FreeFloatingAcc reducedAcc(reducedModel);
        FreeFloatingGeneralizedTorques reducedTrqs(reducedModel);
        FreeFloatingGeneralizedTorques reducedTrqsCheck(reducedModel);

        reducedPos.worldBasePos() = getRandomTransform();
        getRandomVector(reducedPos.jointPos());

        reducedVel.baseVel() = getRandomTwist();
        getRandomVector(reducedVel.jointVel());

        reducedAcc.baseAcc() = getRandomTwist();
        getRandomVector(reducedAcc.jointAcc());

        FreeFloatingVel fullVel(model);
        FreeFloatingAcc fullAcc(model);
        FreeFloatingGeneralizedTorques fullTrqs(model);

        fullPos.worldBasePos() = reducedPos.worldBasePos();
        fullVel.baseVel()      = reducedVel.baseVel();
        fullAcc.baseAcc()      = reducedAcc.baseAcc();

        copyFromReducedToFull(reducedPos.jointPos(),fullPos.jointPos(),reducedModel,model);
        copyFromReducedToFull(reducedVel.jointVel(),fullVel.jointVel(),reducedModel,model);
        copyFromReducedToFull(reducedAcc.jointAcc(),fullAcc.jointAcc(),reducedModel,model);

        fullRNEA.runRNEA(fullPos,fullVel,fullAcc,fullTrqs);
        reducedRNEA.runRNEA(reducedPos,reducedVel,reducedAcc,reducedTrqs);

        reducedTrqsCheck.baseWrench() = fullTrqs.baseWrench();
        copyFromFullToReduced(reducedTrqsCheck.jointTorques(),fullTrqs.jointTorques(),reducedModel,model);

        ASSERT_EQUAL_VECTOR_TOL(reducedTrqs.baseWrench().asVector(),reducedTrqsCheck.baseWrench().asVector(),1e-8);
        ASSERT_EQUAL_VECTOR_TOL(reducedTrqs.jointTorques(),reducedTrqsCheck.jointTorques(),1e-8);
    }

}

void checkExtractSubModel(const Model & model)
{
    for(size_t jnts=1; jnts < model.getNrOfJoints(); jnts += 5)
    {

        Traversal traversal;
        bool ok = model.computeFullTreeTraversal(traversal);

        ASSERT_EQUAL_DOUBLE(ok,true);

        std::vector<std::string> jointInReducedModel;
        getRandomSubsetOfJoints(model,jnts,jointInReducedModel);

        iDynTree::SubModelDecomposition subModels;
        ok = subModels.splitModelAlongJoints(model,traversal,jointInReducedModel);

        ASSERT_EQUAL_DOUBLE(ok,true);

        for(int subModelIdx = 0; subModelIdx < subModels.getNrOfSubModels(); subModelIdx++)
        {
            const Traversal & subModelTraversal = subModels.getTraversal(subModelIdx);

            Model reducedModel;
            ok = extractSubModel(model, subModelTraversal, reducedModel);

            ASSERT_EQUAL_DOUBLE(ok,true);

            for (int linkIdx = 0; linkIdx < subModelTraversal.getNrOfVisitedLinks(); linkIdx++)
            {
                auto linkSubModel = subModelTraversal.getLink(linkIdx);
                auto linkModel = model.getLink(model.getLinkIndex(reducedModel.getLinkName(linkIdx)));

                ASSERT_EQUAL_MATRIX(linkSubModel->getInertia().asMatrix(), linkModel->getInertia().asMatrix());
            }
        }
    }
}

void checkAll(const Model & model)
{
    createCopyAndDestroy(model);
    checkNeighborSanity(model,false);
    checkComputeTraversal(model);
    checkReducedModel(model);
    checkExtractSubModel(model);
}

void checkSimpleModel()
{
    std::cout << "Checking simple model... " << std::endl;

    double rotInertiaData[3*3] = {14.0,0.0,0.0,
                                  0.0,12.0,0.0,
                                  0.0,0.0,10.0};

    SpatialInertia inertiaLink0(1.0,Position(100,0,0),RotationalInertia(rotInertiaData,3,3));

    Link link0;
    link0.setInertia(inertiaLink0);

    Link link1(link0);

    FixedJoint fixJoint(0,1,Transform(Rotation::Identity(),Position(1,3,4)));

    {
        Model model;

        model.addLink("link0",link0);
        model.addLink("link1",link1);

        model.addJoint("fixedJoint",&fixJoint);

        model.addAdditionalFrameToLink("link0", "frame0_0", iDynTree::Transform::Identity());
        model.addAdditionalFrameToLink("link1", "frame1_0", iDynTree::Transform::Identity());
        model.addAdditionalFrameToLink("link1", "frame1_1", iDynTree::Transform::Identity());


        ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),2);
        ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(0),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(1),1);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(0,0).neighborLink,1);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(1,0).neighborLink,0);
        std::vector<FrameIndex> link0_frames;
        model.getLinkAdditionalFrames(0, link0_frames);
        ASSERT_EQUAL_DOUBLE(link0_frames.size(), 1);
        std::vector<FrameIndex> link1_frames;
        model.getLinkAdditionalFrames(1, link1_frames);
        ASSERT_EQUAL_DOUBLE(link1_frames.size(), 2);
        ASSERT_IS_TRUE(link1_frames[0] < link1_frames[1]);

        createCopyAndDestroy(model);
        checkComputeTraversal(model);
    }
}

void checkInsertJointAndLink()
{
    std::cout << "Checking InsertJointAndLink... " << std::endl;

    double rotInertiaData[3*3] = {14.0,0.0,0.0,
                                  0.0,12.0,0.0,
                                  0.0,0.0,10.0};

    SpatialInertia inertiaLink0(1.0,Position(100,0,0),RotationalInertia(rotInertiaData,3,3));

    Link link0;
    link0.setInertia(inertiaLink0);

    Link link1(link0);


    FixedJoint fixJoint(0,1,Transform(Rotation::Identity(),Position(1,3,4)));

    Link linkToInsert(link0);
    FixedJoint newJoint(2,1,Transform(Rotation::Identity(),Position(1,3,4)));
    Transform newTransform(Rotation::Identity(),Position(1,3,4));

    {
        Model model;

        model.addLink("link0",link0);
        model.addLink("link1",link1);

        model.addJoint("fixedJoint",&fixJoint);

        std::cerr << "Inserting link named insertedLink and joint named newJoint "<< std::endl;
        JointIndex ok= model.insertLinkToExistingJointAndAddJointForDisplacedLink("fixedJoint","link0",newTransform,"newJoint",&newJoint,"insertedLink",linkToInsert);
        //std::cerr << "Return value of insert JointAndLink "<<ok << std::endl;
        std::cerr <<model.toString()<< std::endl;
        if (ok==-1)
        {
            ASSERT_EQUAL_DOUBLE(ok,1);
        }
        //std::cout << "Inserted link and joint "<< std::endl;

        ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),3);
        ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),2);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(0),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(1),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(2),2);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(0,0).neighborLink,2);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(1,0).neighborLink,2);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(2,0).neighborLink,0);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(2,1).neighborLink,1);
    }
}

void checkRandomChains()
{
    std::cout << "Checking random chains..." << std::endl;

    for(int i=2; i <= 50; i += 15 )
    {
        Model randomModel = getRandomChain(i);

        std::cout << "Checking reduced model for random chain of size: " << i << std::endl;
        checkAll(randomModel);
    }


}

void checkNormalizeJointPosCoords()
{
    std::cout << "Checking normalizeJointPosCoords..." << std::endl;

    // Test with a random model that includes spherical joints
    Model randomModel = getRandomModel(10, /*nrOfAdditionalFrames =*/5, /*onlyRevoluteJoints=*/false, /*includeSphericalJoints=*/true);

    // Create a position vector with potentially non-normalized values
    FreeFloatingPos pos(randomModel);
    getRandomJointPositions(pos.jointPos(), randomModel);

    // Now deliberately corrupt some joint positions to be non-normalized
    for(JointIndex jntIdx = 0; jntIdx < randomModel.getNrOfJoints(); jntIdx++)
    {
        IJointConstPtr joint = randomModel.getJoint(jntIdx);

        if (joint->getNrOfPosCoords() == 4 && joint->getNrOfDOFs() == 3)
        {
            // This appears to be a spherical joint - set non-normalized quaternion values
            size_t offset = joint->getPosCoordsOffset();
            pos.jointPos()(offset + 0) = 2.0;  // Non-normalized quaternion
            pos.jointPos()(offset + 1) = 3.0;
            pos.jointPos()(offset + 2) = 4.0;
            pos.jointPos()(offset + 3) = 5.0;
        }
        else if (joint->getNrOfPosCoords() == 2 && joint->getNrOfDOFs() == 1)
        {
            // This might be a RevoluteSO2 joint - set non-normalized complex values
            size_t offset = joint->getPosCoordsOffset();
            pos.jointPos()(offset + 0) = 2.0;  // Non-normalized complex number
            pos.jointPos()(offset + 1) = 3.0;
        }
    }

    // Store the original values for comparison
    JointPosDoubleArray originalPos = pos.jointPos();

    // Normalize all joint position coordinates
    for(JointIndex jntIdx = 0; jntIdx < randomModel.getNrOfJoints(); jntIdx++)
    {
        IJointConstPtr joint = randomModel.getJoint(jntIdx);
        size_t nrOfPosCoords = joint->getNrOfPosCoords();

        if (nrOfPosCoords > 0)
        {
            size_t jointOffset = joint->getPosCoordsOffset();
            iDynTree::Span<double> jointSpan(pos.jointPos().data() + jointOffset, nrOfPosCoords);
            bool success = joint->normalizeJointPosCoords(jointSpan);
            ASSERT_EQUAL_DOUBLE(success, true);
        }
    }

    // Verify that spherical joints (quaternions) are now normalized
    for(JointIndex jntIdx = 0; jntIdx < randomModel.getNrOfJoints(); jntIdx++)
    {
        IJointConstPtr joint = randomModel.getJoint(jntIdx);

        if (joint->getNrOfPosCoords() == 4 && joint->getNrOfDOFs() == 3)
        {
            // Check that quaternion is normalized
            size_t offset = joint->getPosCoordsOffset();
            double w = pos.jointPos()(offset + 0);
            double x = pos.jointPos()(offset + 1);
            double y = pos.jointPos()(offset + 2);
            double z = pos.jointPos()(offset + 3);
            double norm = std::sqrt(w*w + x*x + y*y + z*z);
            ASSERT_EQUAL_DOUBLE_TOL(norm, 1.0, 1e-10);
        }
        else if (joint->getNrOfPosCoords() == 2 && joint->getNrOfDOFs() == 1)
        {
            // Check that complex number is normalized (for RevoluteSO2)
            size_t offset = joint->getPosCoordsOffset();
            double real = pos.jointPos()(offset + 0);
            double imag = pos.jointPos()(offset + 1);
            double norm = std::sqrt(real*real + imag*imag);
            ASSERT_EQUAL_DOUBLE_TOL(norm, 1.0, 1e-10);
        }
    }

    std::cout << "normalizeJointPosCoords test passed!" << std::endl;
}

void checkRandomModels()
{
    std::cout << "Checking random models..." << std::endl;

    for(int i=2; i <= 100; i += 30 )
    {
        Model randomModel = getRandomModel(i,/*nrOfAdditionalFrames =*/10, /*onlyRevoluteJoints=*/false, /*includeSphericalJoints=*/true);

        std::cout << "Checking reduced model for random model of size: " << i << std::endl;
        checkAll(randomModel);
    }

}

int main()
{
    checkSimpleModel();
    checkRandomChains();
    checkRandomModels();
    checkNormalizeJointPosCoords();
    checkInsertJointAndLink();
    return EXIT_SUCCESS;
}
