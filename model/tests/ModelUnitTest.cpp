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

#include <cassert>
#include <cstdio>
#include <cstdlib>

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
    for(int link =0; link < model.getNrOfLinks(); link++ )
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
        createCopyAndDestroy(randomModel);
        checkNeighborSanity(randomModel,false);
        checkComputeTraversal(randomModel);
    }

}

void checkRandomModels()
{
    std::cout << "Checking random models..." << std::endl;

    for(int i=2; i <= 100; i += 10 )
    {
        Model randomModel = getRandomModel(i);

        std::cout << "Checking random model of size: " << i << std::endl;
        createCopyAndDestroy(randomModel);
        checkNeighborSanity(randomModel,false);
        checkComputeTraversal(randomModel);
    }

}

int main()
{
    checkSimpleModel();
    checkRandomChains();
    checkRandomModels();

    return EXIT_SUCCESS;
}
