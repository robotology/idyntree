/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MODEL_TEST_UTILS_H
#define IDYNTREE_MODEL_TEST_UTILS_H


#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/LinkState.h>

#include <iDynTree/Core/TestUtils.h>

#include <cassert>

namespace iDynTree
{

inline Link getRandomLink()
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
inline void addRandomLinkToModel(Model & model, std::string parentLink, std::string newLinkName)
{
    // Add Link
    LinkIndex newLinkIndex = model.addLink(newLinkName,getRandomLink());

    // Now add joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLink);

    int nrOfJointTypes = 2;

    int jointType = rand() % nrOfJointTypes;

    jointType = 1;

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

/**
 * Add a random additional frame to a model model.
 */
inline void addRandomAdditionalFrameToModel(Model & model, std::string parentLink, std::string newFrameName)
{
    model.addAdditionalFrameToLink(parentLink,newFrameName,getRandomTransform());
}

inline LinkIndex getRandomLinkIndexOfModel(const Model & model)
{
    int nrOfLinks = model.getNrOfLinks();

    return rand() % nrOfLinks;
}

inline std::string getRandomLinkOfModel(const Model & model)
{
    LinkIndex randomLink = getRandomLinkIndexOfModel(model);

    return model.getLinkName(randomLink);
}

inline std::string int2string(int i)
{
    std::stringstream ss;

    ss << i;

    return ss.str();
}

inline Model getRandomModel(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName);
    }

    for(unsigned int i=0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + int2string(i);
        addRandomAdditionalFrameToModel(model,parentLink,frameName);
    }

    return model;
}

inline Model getRandomChain(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    std::string linkName = "baseLink";
    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = linkName;
        linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName);
    }

    for(unsigned int i=0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + int2string(i);
        addRandomAdditionalFrameToModel(model,parentLink,frameName);
    }

    return model;
}

/**
 * Get random robot positions, velocities and accelerations
 * and external wrenches to be given as an input to InverseDynamics.
 */
inline bool getRandomInverseDynamicsInputs(FreeFloatingPos& pos,
                                           FreeFloatingVel& vel,
                                           FreeFloatingAcc& acc,
                                           LinkNetExternalWrenches& extWrenches)
{
    pos.worldBasePos() =  getRandomTransform();
    vel.baseVel() = getRandomTwist();
    acc.baseAcc() =  getRandomTwist();


    for(unsigned int jnt=0; jnt < pos.getNrOfPosCoords(); jnt++)
    {
        pos.jointPos()(jnt) = getRandomDouble();
    }

    for(unsigned int jnt=0; jnt < vel.getNrOfDOFs(); jnt++)
    {
        vel.jointVel()(jnt) = getRandomDouble();
        acc.jointAcc()(jnt) = getRandomDouble();
    }

    return true;
}

}

#endif /* IDYNTREE_MODEL_TEST_UTILS_H */