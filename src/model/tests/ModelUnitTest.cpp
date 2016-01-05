/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/Core/TestUtils.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>

// For modelTransformsers testing
#include <iDynTree/Model/ModelTransformers.h>
#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>
#include <iDynTree/Model/FreeFloatingState.h>

using namespace iDynTree;


void createCopyAndDestroy(const Model & model)
{
    Model * p_model = new Model(model);
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    *p_model = model;
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfLinks(),model.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(p_model->getNrOfJoints(),model.getNrOfJoints());
    delete p_model;
}

void checkComputeTraversal(const Model & model)
{
    Traversal traversal;
    bool ok = model.computeFullTreeTraversal(traversal);

    ASSERT_EQUAL_DOUBLE(ok,true);
    ASSERT_EQUAL_DOUBLE(traversal.getNrOfVisitedLinks(),model.getNrOfLinks());
}

Link getRandomLink()
{
    double cxx = getRandomDouble(0,3);
    double cyy = getRandomDouble(0,4);
    double czz = getRandomDouble(0,6);
    double rotInertiaData[3*3] = {czz+cyy,0.0,0.0,
                                  0.0,cxx+czz,0.0,
                                  0.0,0.0,cxx+cyy};

    Rotation rot = Rotation::RPY(getRandomDouble(),getRandomDouble(-1,1),getRandomDouble());

    SpatialInertia inertiaLink(getRandomDouble(0,4),
                               Position(getRandomDouble(-2,2),getRandomDouble(-2,2),getRandomDouble(-2,2)),
                               rot*RotationalInertiaRaw(rotInertiaData,3,3));

    Link link;

    link.setInertia(inertiaLink);

    return link;
}

/**
 * Add a random link with random model.
 */
void addRandomLinkToModel(Model & model, std::string parentLink, std::string newLinkName)
{
    // Add Link
    LinkIndex newLinkIndex = model.addLink(newLinkName,getRandomLink());

    // Now add joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLink);

    int nrOfJointTypes = 2;

    int jointType = rand() % nrOfJointTypes;

    if( jointType == 0 )
    {
        FixedJoint fixJoint(parentLinkIndex,newLinkIndex,getRandomTransform());
        model.addJoint(newLinkName+"joint",&fixJoint);
    }
    else if( jointType == 1 )
    {
        RevoluteJoint revJoint(parentLinkIndex,newLinkIndex,getRandomTransform(),getRandomAxis());
        model.addJoint(newLinkName+"joint",&revJoint);
    }
    else
    {
        assert(false);
    }
}

std::string getRandomLinkOfModel(const Model & model)
{
    int nrOfLinks = model.getNrOfLinks();

    LinkIndex randomLink = rand() % nrOfLinks;

    return model.getLinkName(randomLink);
}

std::string int2string(int i)
{
    std::stringstream ss;

    ss << i;

    return ss.str();
}

Model getRandomModel(unsigned int nrOfJoints)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName);
    }

    return model;
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

Model getRandomChain(unsigned int nrOfJoints)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    std::string linkName = "baseLink";
    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = linkName;
        linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName);

        checkNeighborSanity(model,false);
    }

    return model;
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
                                    pos,linkVels,linkAccs,
                                    linkExtF,linkIntF,trqs);

        return ok;
    }
};

/**
 * Copy a vector of the dofs from the reduced model to a full model.
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

            fullVector(jntInFullModel->getPosCoordsOffset()) = reducedVector(jnt->getPosCoordsOffset());
        }
    }
}


/**
 * Copy a vector of the dofs from the reduced model to a full model.
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

            reducedVector(jnt->getPosCoordsOffset()) = fullVector(jntInFullModel->getPosCoordsOffset());
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
        std::vector<std::string> jointInReducedModel;
        getRandomSubsetOfJoints(model,jnts,jointInReducedModel);

        Model reducedModel;
        bool ok = createReducedModel(model,jointInReducedModel,reducedModel);

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

        FreeFloatingPos fullPos(model);
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

void checkAll(const Model & model)
{
    createCopyAndDestroy(model);
    checkNeighborSanity(model,false);
    checkComputeTraversal(model);
}

void checkSimpleModel()
{
    std::cout << "Checking simple model... " << std::endl;

    double rotInertiaData[3*3] = {14.0,0.0,0.0,
                                  0.0,12.0,0.0,
                                  0.0,0.0,10.0};

    SpatialInertia inertiaLink0(1.0,Position(100,0,0),RotationalInertiaRaw(rotInertiaData,3,3));

    Link link0;
    link0.setInertia(inertiaLink0);

    Link link1(link0);

    FixedJoint fixJoint(0,1,Transform(Rotation::Identity(),Position(1,3,4)));

    {
        Model model;

        model.addLink("link0",link0);
        model.addLink("link1",link1);

        model.addJoint("fixedJoint",&fixJoint);

        ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(),2);
        ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(0),1);
        ASSERT_EQUAL_DOUBLE(model.getNrOfNeighbors(1),1);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(0,0).neighborLink,1);
        ASSERT_EQUAL_DOUBLE(model.getNeighbor(1,0).neighborLink,0);

        createCopyAndDestroy(model);
        checkComputeTraversal(model);
    }
}

void checkRandomChains()
{
    std::cout << "Checking random chains..." << std::endl;


    for(int i=4; i <= 100; i += 10)
    {
        Model randomModel = getRandomChain(i);

        std::cout << "Checking random chain of size: " << i << std::endl;
        checkAll(randomModel);
    }

    for(int i=2; i <= 100; i += 30 )
    {
        Model randomModel = getRandomChain(i);

        std::cout << "Checking reduced model for random chain of size: " << i << std::endl;
        checkAll(randomModel);
    }


}

void checkRandomModels()
{
    std::cout << "Checking random models..." << std::endl;

    for(int i=2; i <= 100; i += 10 )
    {
        Model randomModel = getRandomModel(i);

        std::cout << "Checking random model of size: " << i << std::endl;
        checkReducedModel(randomModel);
    }

    for(int i=2; i <= 100; i += 30 )
    {
        Model randomModel = getRandomModel(i);

        std::cout << "Checking reduced model for random model of size: " << i << std::endl;
        checkReducedModel(randomModel);
    }

}

int main()
{
    checkSimpleModel();
    checkRandomChains();
    checkRandomModels();

    return EXIT_SUCCESS;
}
