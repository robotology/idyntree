// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_TEST_UTILS_H
#define IDYNTREE_MODEL_TEST_UTILS_H


#include <iDynTree/Model.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/LinkState.h>

#include <iDynTree/TestUtils.h>

#include <cassert>
#include "IJoint.h"

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
                               rot*RotationalInertia(rotInertiaData,3,3));

    Link link;

    link.setInertia(inertiaLink);

    return link;
}

/**
 * Add a random link with random model.
 */
inline void addRandomLinkToModel(Model & model, std::string parentLink, std::string newLinkName, bool onlyRevoluteJoints=false)
{
    // Add Link
    LinkIndex newLinkIndex = model.addLink(newLinkName,getRandomLink());

    // Now add joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLink);

    int nrOfJointTypes = 3;

    int jointType = rand() % nrOfJointTypes;

    if (onlyRevoluteJoints)
    {
        jointType = 1;
    }

    if( jointType == 0 )
    {
        FixedJoint fixJoint(parentLinkIndex,newLinkIndex,getRandomTransform());
        model.addJoint(newLinkName+"joint",&fixJoint);
    }
    else if( jointType == 1 )
    {
        RevoluteJoint revJoint;
        revJoint.setAttachedLinks(parentLinkIndex,newLinkIndex);
        revJoint.setRestTransform(getRandomTransform());
        revJoint.setAxis(getRandomAxis(),newLinkIndex);
        model.addJoint(newLinkName+"joint",&revJoint);
    }
    else if( jointType == 2 )
    {
        PrismaticJoint prismJoint;
        prismJoint.setAttachedLinks(parentLinkIndex,newLinkIndex);
        prismJoint.setRestTransform(getRandomTransform());
        prismJoint.setAxis(getRandomAxis(),newLinkIndex);
        model.addJoint(newLinkName+"joint",&prismJoint);
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

inline Model getRandomModel(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10, bool onlyRevoluteJoints=false)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName,onlyRevoluteJoints);
    }

    for(unsigned int i=0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + int2string(i);
        addRandomAdditionalFrameToModel(model,parentLink,frameName);
    }

    return model;
}

inline Model getRandomChain(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10, bool onlyRevoluteJoints=false)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    std::string linkName = "baseLink";
    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = linkName;
        linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName,onlyRevoluteJoints);
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
 * Get random joint position consistently with the limits of the model.
 * If the input vector has the wrong size, it will be resized.
 */
inline void getRandomJointPositions(VectorDynSize& vec, const Model& model)
{
    vec.resize(model.getNrOfPosCoords());
    for(JointIndex jntIdx=0; jntIdx < model.getNrOfJoints(); jntIdx++)
    {
        IJointConstPtr jntPtr = model.getJoint(jntIdx);
        if( jntPtr->hasPosLimits() )
        {
            for(int i=0; i < jntPtr->getNrOfPosCoords(); i++)
            {
                double max = jntPtr->getMaxPosLimit(i);
                double min = jntPtr->getMinPosLimit(i);
                vec(jntPtr->getDOFsOffset()+i) = getRandomDouble(min,max);
            }
        }
        else
        {
            for(int i=0; i < jntPtr->getNrOfPosCoords(); i++)
            {
                vec(jntPtr->getDOFsOffset()+i) = getRandomDouble();
            }
        }
    }

    return;
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
    pos.worldBasePos() = getRandomTransform();
    vel.baseVel() =  getRandomTwist();
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
