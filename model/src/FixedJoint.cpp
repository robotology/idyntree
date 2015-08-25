/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Utils.h>

#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/IJointStateInterfaces.h>
#include <iDynTree/Model/LinkState.h>

#include <cassert>

namespace iDynTree
{

FixedJoint::FixedJoint(const LinkIndex _link1, const LinkIndex _link2,
                       const Transform& _link1_X_link2):
                       link1(_link1), link2(_link2),
                       link1_X_link2(_link1_X_link2)
{
}

FixedJoint::FixedJoint(const FixedJoint& other):
                       link1(other.link1), link2(other.link2),
                       link1_X_link2(other.link1_X_link2)
{
}


FixedJoint::~FixedJoint()
{

}

IJoint * FixedJoint::clone() const
{
    return (IJoint *) new FixedJoint(*this);
}

void FixedJoint::setAttachedLinks(const LinkIndex _link1, const LinkIndex _link2)
{
    link1 = _link1;
    link2 = _link2;
    return;
}

void FixedJoint::setRestTransform(const Transform& _link1_X_link2)
{
    link1_X_link2 = _link1_X_link2;
}

unsigned int FixedJoint::getNrOfPosCoords() const
{
    return 0;
}

unsigned int FixedJoint::getNrOfDOFs() const
{
    return 0;
}

LinkIndex FixedJoint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex FixedJoint::getSecondAttachedLink() const
{
    return link2;
}



Transform FixedJoint::getTransform(const IJointPos& state, const LinkIndex p_linkA, const LinkIndex p_linkB) const
{
    if( p_linkA == this->link1 )
    {
        iDynTreeAssert(p_linkB == this->link2);
        return this->link1_X_link2;
    }
    else
    {
        iDynTreeAssert(p_linkA == this->link2);
        iDynTreeAssert(p_linkB == this->link1);
        return this->link1_X_link2.inverse();
    }
}

void FixedJoint::computeJointTorque(const IJointPos & state, const Wrench& internalWrench,
                                    LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied,
                                    IJointTorque& outputTorque) const
{
    // A fixed joint would have a torque of size 0
    return;
}

LinkPosVelAcc FixedJoint::computeLinkPosVelAcc(const IJointPosVelAcc& state,
                                           const LinkPosVelAcc& linkBstate,
                                           const LinkIndex linkA, const LinkIndex linkB) const
{
    LinkPosVelAcc linkAstate;
    Transform a_X_b = this->getTransform(state,linkA,linkB);
    Transform b_X_a = this->getTransform(state,linkB,linkA);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_T_a  = ref_T_b*b_
    linkAstate.pos() = linkBstate.pos()*b_X_a;

    // Propagate twist and spatial acceleration: for a fixed joint the twist of two attached links is the same,
    // expect that they are usually expressed in different frames
    linkAstate.vel() = a_X_b*linkBstate.vel();
    linkAstate.acc() = a_X_b*linkBstate.acc();

    return linkAstate;
}

LinkVelAcc FixedJoint::computeLinkVelAcc(const IJointPosVelAcc& state, const LinkVelAcc& linkBstate,
                                     const LinkIndex linkA, const LinkIndex linkB) const
{
    LinkVelAcc linkAstate;
    Transform a_X_b = this->getTransform(state,linkA,linkB);
    // Propagate twist: for a fixed joint the twist of two attached links is the same,
    // expect that they are usually expressed in different frames
    linkAstate.vel() = a_X_b*linkBstate.vel();
    linkAstate.acc() = a_X_b*linkBstate.acc();

    return linkAstate;
}


}
