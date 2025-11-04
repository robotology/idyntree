// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODEL_TEST_UTILS_H
#define IDYNTREE_MODEL_TEST_UTILS_H


#include <iDynTree/Model.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>
#include <iDynTree/RevoluteSO2Joint.h>
#include <iDynTree/PrismaticJoint.h>
#include <iDynTree/SphericalJoint.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/LinkState.h>

#include <iDynTree/TestUtils.h>

#include <cassert>
#include <cmath>
#include "IJoint.h"

namespace iDynTree
{

/**
 * Bitfield enum for selecting joint types in random model generation.
 */
enum JointTypes : unsigned int
{
    JOINT_FIXED = 1 << 0,        // 0001 - FixedJoint
    JOINT_REVOLUTE = 1 << 1,     // 0010 - RevoluteJoint
    JOINT_PRISMATIC = 1 << 2,    // 0100 - PrismaticJoint
    JOINT_REVOLUTE_SO2 = 1 << 3, // 1000 - RevoluteSO2Joint
    JOINT_SPHERICAL = 1 << 4     // 10000 - SphericalJoint
};

// Simple joint types: Fixed, Revolute, and Prismatic
const unsigned int SIMPLE_JOINT_TYPES = JOINT_FIXED | JOINT_REVOLUTE | JOINT_PRISMATIC;

// All available joint types: Fixed, Revolute, Prismatic, RevoluteSO2, and Spherical
const unsigned int ALL_JOINT_TYPES = JOINT_FIXED | JOINT_REVOLUTE | JOINT_PRISMATIC | JOINT_REVOLUTE_SO2 | JOINT_SPHERICAL;

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
inline void addRandomLinkToModel(Model & model, std::string parentLink, std::string newLinkName, unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES)
{
    // Add Link
    LinkIndex newLinkIndex = model.addLink(newLinkName,getRandomLink());

    // Now add joint
    LinkIndex parentLinkIndex = model.getLinkIndex(parentLink);

    // Collect available joint types based on the bitfield
    std::vector<int> availableJointTypes;
    if (allowedJointTypes & JOINT_FIXED) {
        availableJointTypes.push_back(0);
    }
    if (allowedJointTypes & JOINT_REVOLUTE) {
        availableJointTypes.push_back(1);
    }
    if (allowedJointTypes & JOINT_PRISMATIC) {
        availableJointTypes.push_back(2);
    }
    if (allowedJointTypes & JOINT_REVOLUTE_SO2) {
        availableJointTypes.push_back(3);
    }
    if (allowedJointTypes & JOINT_SPHERICAL) {
        availableJointTypes.push_back(4);
    }

    // If no joint types are allowed, use default
    if (availableJointTypes.empty()) {
        allowedJointTypes = SIMPLE_JOINT_TYPES;
        if (allowedJointTypes & JOINT_FIXED) {
            availableJointTypes.push_back(0);
        }
        if (allowedJointTypes & JOINT_REVOLUTE) {
            availableJointTypes.push_back(1);
        }
        if (allowedJointTypes & JOINT_PRISMATIC) {
            availableJointTypes.push_back(2);
        }
    }

    // Select a random joint type from available ones
    int jointTypeIndex = rand() % availableJointTypes.size();
    int jointType = availableJointTypes[jointTypeIndex];

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
    else if( jointType == 3 )
    {
        RevoluteSO2Joint revSO2Joint;
        revSO2Joint.setAttachedLinks(parentLinkIndex,newLinkIndex);
        revSO2Joint.setRestTransform(getRandomTransform());
        revSO2Joint.setAxis(getRandomAxis(),newLinkIndex);
        model.addJoint(newLinkName+"joint",&revSO2Joint);
    }
    else if( jointType == 4 )
    {
        SphericalJoint sphericalJoint;
        sphericalJoint.setAttachedLinks(parentLinkIndex,newLinkIndex);
        sphericalJoint.setRestTransform(getRandomTransform());
        sphericalJoint.setJointCenter(newLinkIndex,getRandomPosition());
        model.addJoint(newLinkName+"joint",&sphericalJoint);
    }
    else
    {
        assert(false);
    }
}

/**
 * Add a random link with random model.
 *
 * This version is deprecated as it can only permit to either add revolute, prismatic and fixed joints 
 * (when onlyRevoluteJoints=false) or only revolute (when onlyRevoluteJoints=true). Please migrate to
 * use addRandomLinkToModel in which the fourth argument is an unsigned int
 */
IDYNTREE_DEPRECATED_WITH_MSG("Use addRandomLinkToModel variant that takes in input the allowedJointTypes as bitset.")
inline void addRandomLinkToModel(Model & model, std::string parentLink, std::string newLinkName, bool onlyRevoluteJoints)
{
    unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES;
    if (onlyRevoluteJoints) 
    {
        allowedJointTypes = JOINT_REVOLUTE;
    }
    return addRandomLinkToModel(model, parentLink, newLinkName, allowedJointTypes);
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

/**
 * Generate a random model with the specified number of joints and additional frames.
 *
 * @param nrOfJoints Number of joints to add to the model
 * @param nrOfAdditionalFrames Number of additional frames to add (default: 10)
 * @param allowedJointTypes Bitfield specifying which joint types to include (default: Fixed, Revolute, Prismatic)
 *
 * Example usage:
 * - getRandomModel(5) // Uses default joint types (Fixed, Revolute, Prismatic)
 * - getRandomModel(5, 10, JOINT_REVOLUTE | JOINT_PRISMATIC) // Only revolute and prismatic joints
 * - getRandomModel(5, 10, JOINT_REVOLUTE) // Only revolute joints
 * - getRandomModel(5, 10, SIMPLE_JOINT_TYPES | JOINT_REVOLUTE_SO2) // All joint types including SO2
 * - getRandomModel(5, 10, ALL_JOINT_TYPES) // All available joint types including Spherical
 */
inline Model getRandomModel(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10, unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName,allowedJointTypes);
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
 * Generate a random chain (sequential links) with the specified number of joints and additional frames.
 *
 * @param nrOfJoints Number of joints to add to the chain
 * @param nrOfAdditionalFrames Number of additional frames to add (default: 10)
 * @param allowedJointTypes Bitfield specifying which joint types to include (default: Fixed, Revolute, Prismatic)
 */
inline Model getRandomChain(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10, unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES)
{
    Model model;

    model.addLink("baseLink",getRandomLink());

    std::string linkName = "baseLink";
    for(unsigned int i=0; i < nrOfJoints; i++)
    {
        std::string parentLink = linkName;
        linkName = "link" + int2string(i);
        addRandomLinkToModel(model,parentLink,linkName,allowedJointTypes);
    }

    for(unsigned int i=0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + int2string(i);
        addRandomAdditionalFrameToModel(model,parentLink,frameName);
    }

    return model;
}

// Backward compatibility overloads
IDYNTREE_DEPRECATED_WITH_MSG("Use getRandomModel variant that takes in input the allowedJointTypes as bitset.")
inline Model getRandomModel(unsigned int nrOfJoints, size_t nrOfAdditionalFrames, bool onlyRevoluteJoints)
{
    unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES;
    if (onlyRevoluteJoints) {
        allowedJointTypes = JOINT_REVOLUTE;
    }
    return getRandomModel(nrOfJoints, nrOfAdditionalFrames, allowedJointTypes);
}

IDYNTREE_DEPRECATED_WITH_MSG("Use getRandomChain variant that takes in input the allowedJointTypes as bitset.")
inline Model getRandomChain(unsigned int nrOfJoints, size_t nrOfAdditionalFrames, bool onlyRevoluteJoints)
{
    unsigned int allowedJointTypes = SIMPLE_JOINT_TYPES;
    if (onlyRevoluteJoints) {
        allowedJointTypes = JOINT_REVOLUTE;
    }
    return getRandomChain(nrOfJoints, nrOfAdditionalFrames, allowedJointTypes);
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
                vec(jntPtr->getPosCoordsOffset()+i) = getRandomDouble(min,max);
            }
        }
        else
        {
            // Set random values for all position coordinates
            for(int i=0; i < jntPtr->getNrOfPosCoords(); i++)
            {
                vec(jntPtr->getPosCoordsOffset()+i) = getRandomDouble();
            }
        }

        // Use normalizeJointPosCoords to make the code more general
        // This handles different joint types (RevoluteSO2, Revolute, Prismatic, etc.) appropriately
        jntPtr->normalizeJointPosCoords(vec);
    }

    return;
}

/**
 * Get random robot positions, velocities and accelerations
 * and external wrenches to be given as an input to InverseDynamics.
 *
 * @deprecated This function does not work properly with RevoluteSO2Joint joints.
 * Use the version that takes a Model parameter instead.
 */
IDYNTREE_DEPRECATED_WITH_MSG("This function does not work properly with RevoluteSO2Joint joints. Use the version that takes a Model parameter instead.")
inline bool getRandomInverseDynamicsInputs(FreeFloatingPos& pos,
                                           FreeFloatingVel& vel,
                                           FreeFloatingAcc& acc,
                                           LinkNetExternalWrenches& extWrenches)
{
    pos.worldBasePos() = getRandomTransform();
    vel.baseVel() =  getRandomTwist();
    acc.baseAcc() =  getRandomTwist();

    // Simple random generation - does not work properly with RevoluteSO2Joint
    for(unsigned int jnt=0; jnt < pos.getNrOfPosCoords(); jnt++)
    {
        pos.jointPos()(jnt) = getRandomDouble();
    }

    for(unsigned int jnt=0; jnt < vel.getNrOfDOFs(); jnt++)
    {
        vel.jointVel()(jnt) = getRandomDouble();
        acc.jointAcc()(jnt) = getRandomDouble();
    }

    for(unsigned int link=0; link < extWrenches.getNrOfLinks(); link++ )
    {
        extWrenches(link) = getRandomWrench();
    }

    return true;
}

/**
 * Get random robot positions, velocities and accelerations
 * and external wrenches to be given as an input to InverseDynamics.
 */
inline bool getRandomInverseDynamicsInputs(const Model& model,
                                           FreeFloatingPos& pos,
                                           FreeFloatingVel& vel,
                                           FreeFloatingAcc& acc,
                                           LinkNetExternalWrenches& extWrenches)
{
    pos.worldBasePos() = getRandomTransform();
    vel.baseVel() =  getRandomTwist();
    acc.baseAcc() =  getRandomTwist();

    // Use model-aware joint position generation for proper handling of all joint types
    getRandomJointPositions(pos.jointPos(), model);

    for(unsigned int jnt=0; jnt < vel.getNrOfDOFs(); jnt++)
    {
        vel.jointVel()(jnt) = getRandomDouble();
        acc.jointAcc()(jnt) = getRandomDouble();
    }

    return true;
}

}

#endif /* IDYNTREE_MODEL_TEST_UTILS_H */
