/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/RevoluteJoint.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Model/IJointStateInterfaces.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>

#include <cassert>

namespace iDynTree
{

RevoluteJoint::RevoluteJoint(const LinkIndex _link1, const LinkIndex _link2,
                             const Transform& _link1_X_link2, const Axis& _rotation_axis_wrt_link1):
                             link1(_link1), link2(_link2), link1_X_link2_at_rest(_link1_X_link2),
                             rotation_axis_wrt_link1(_rotation_axis_wrt_link1)
{
}

RevoluteJoint::RevoluteJoint(const RevoluteJoint& other):
                       link1(other.link1), link2(other.link2),
                       link1_X_link2_at_rest(other.link1_X_link2_at_rest),
                       rotation_axis_wrt_link1(other.rotation_axis_wrt_link1)
{
}

RevoluteJoint::~RevoluteJoint()
{
}

IJoint* RevoluteJoint::clone() const
{
    return (IJoint *) new RevoluteJoint(*this);
}


void RevoluteJoint::setRestTransform(const Transform& _link1_X_link2)
{
    this->link1_X_link2_at_rest = _link1_X_link2;
}

LinkIndex RevoluteJoint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex RevoluteJoint::getSecondAttachedLink() const
{
    return link2;
}



Transform RevoluteJoint::getTransform(const IJointPos& state, const LinkIndex p_linkA, const LinkIndex p_linkB) const
{
    assert(state.getNrOfPosCoords() == this->getNrOfPosCoords());
    const double ang = state.pos()(0);
    if( p_linkA == link1 )
    {
        assert(p_linkB == link2);
        return rotation_axis_wrt_link1.getRotationTransform(ang)*link1_X_link2_at_rest;
    }
    else
    {
        assert(p_linkA == link2);
        assert(p_linkB == link1);
        return (rotation_axis_wrt_link1.getRotationTransform(ang)*link1_X_link2_at_rest).inverse();
    }
}


Axis RevoluteJoint::getAxis(const LinkIndex linkA) const
{
    if( linkA == link1 )
    {
        return rotation_axis_wrt_link1;
    }
    else
    {
        assert(linkA == link2);
        return link1_X_link2_at_rest.inverse()*rotation_axis_wrt_link1;
    }
}

void RevoluteJoint::setAttachedLinks(const LinkIndex _link1, const LinkIndex _link2)
{
    this->link1 = _link1;
    this->link2 = _link2;
}

void RevoluteJoint::setAxis(const Axis& revoluteAxis_wrt_link1)
{
    this->rotation_axis_wrt_link1 = revoluteAxis_wrt_link1;
}

LinkVelAcc RevoluteJoint::computeLinkVelAcc(const IJointPosVelAcc& state, const LinkVelAcc& linkBstate,
                                            const LinkIndex linkA, const LinkIndex linkB) const
{
    LinkVelAcc linkAstate;

    double ang = state.pos()(0);
    double dang = state.vel()(0);
    double d2ang = state.acc()(0);

    Transform a_X_b = this->getTransform(state,linkA,linkB);

    // Propagate twist and spatial acceleration: for a revolute joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    Twist vJ_link1 = rotation_axis_wrt_link1.getRotationTwist(dang);
    SpatialAcc aJ_link1 = rotation_axis_wrt_link1.getRotationSpatialAcc(dang);

    if( linkB == link1 )
    {
        linkAstate.vel() = a_X_b*(linkBstate.vel() + vJ_link1);
        Twist & va = linkAstate.vel();
        linkAstate.acc() = a_X_b*(linkBstate.acc() + aJ_link1) + va*(a_X_b*vJ_link1);
    }
    else
    {
        linkAstate.vel() = a_X_b*linkBstate.vel() - vJ_link1;
        Twist & va = linkAstate.vel();
        linkAstate.acc() = a_X_b*linkBstate.acc() - aJ_link1 - va*vJ_link1;
    }

    return linkAstate;
}

LinkPosVelAcc RevoluteJoint::computeLinkPosVelAcc(const IJointPosVelAcc& state, const LinkPosVelAcc& linkBstate,
                                              const LinkIndex linkA, const LinkIndex linkB) const
{
    LinkPosVelAcc linkAstate;

    double ang = state.pos()(0);
    double dang = state.vel()(0);
    double d2ang = state.acc()(0);

    Transform a_X_b = this->getTransform(state,linkA,linkB);
    Transform b_X_a = this->getTransform(state,linkB,linkA);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_T_a  = ref_T_b*b_
    linkAstate.pos() = linkBstate.pos()*b_X_a;

    // Propagate twist and spatial acceleration: for a revolute joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    Twist vJ_link1 = rotation_axis_wrt_link1.getRotationTwist(dang);
    SpatialAcc aJ_link1 = rotation_axis_wrt_link1.getRotationSpatialAcc(dang);

    if( linkB == link1 )
    {
        linkAstate.vel() = a_X_b*(linkBstate.vel() + vJ_link1);
        Twist & va = linkAstate.vel();
        linkAstate.acc() = a_X_b*(linkBstate.acc() + aJ_link1) + va*(a_X_b*vJ_link1);
    }
    else
    {
        linkAstate.vel() = a_X_b*linkBstate.vel() - vJ_link1;
        Twist & va = linkAstate.vel();
        linkAstate.acc() = a_X_b*linkBstate.acc() - aJ_link1 - va*vJ_link1;
    }

    return linkAstate;
}


void RevoluteJoint::computeJointTorque(const IJointPos & state, const Wrench& internalWrench,
                                       LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied,
                                       iDynTree::IJointTorque& outputTorque) const
{
    double & tau = outputTorque.torque()(0);

    if( linkOnWhichWrenchIsApplied == link2 )
    {
        // in this case link2 is the child and link1 is the parent
        iDynTree::Twist S = (link1_X_link2_at_rest.inverse()*rotation_axis_wrt_link1).getRotationTwist(1.0);
        tau = S.dot(internalWrench);
    }
    else
    {
         // in this case link1 is the child and link2 is the parent
        iDynTree::Twist S = -(rotation_axis_wrt_link1).getRotationTwist(1.0);
        tau = S.dot(internalWrench);
    }

}







}
