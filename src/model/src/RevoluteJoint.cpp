/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Model/RevoluteJoint.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/VectorDynSize.h>
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
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(0);
}

RevoluteJoint::RevoluteJoint(const RevoluteJoint& other):
                             link1(other.link1), link2(other.link2),
                             link1_X_link2_at_rest(other.link1_X_link2_at_rest),
                             rotation_axis_wrt_link1(other.rotation_axis_wrt_link1)
{
    this->setPosCoordsOffset(other.getPosCoordsOffset());
    this->setDOFsOffset(other.getDOFsOffset());

    this->resetAxisBuffers();
    this->resetBuffers(0);
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

    this->resetAxisBuffers();
}

LinkIndex RevoluteJoint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex RevoluteJoint::getSecondAttachedLink() const
{
    return link2;
}

Transform RevoluteJoint::getRestTransform(const LinkIndex p_linkA, const LinkIndex p_linkB) const
{
    if( p_linkA == link1 )
    {
        assert(p_linkB == link2);
        return link1_X_link2_at_rest;
    }
    else
    {
        assert(p_linkA == link2);
        assert(p_linkB == link1);
        return link1_X_link2_at_rest.inverse();
    }
}

void RevoluteJoint::updateBuffers(const double new_q) const
{
    if( new_q != q_previous )
    {
        resetBuffers(new_q);
    }

    return;
}

void RevoluteJoint::resetBuffers(const double new_q) const
{
    this->link1_X_link2 = rotation_axis_wrt_link1.getRotationTransform(new_q)*link1_X_link2_at_rest;
    this->link2_X_link1 = link1_X_link2.inverse();

    q_previous = new_q;
}

void RevoluteJoint::resetAxisBuffers() const
{
    this->S_link1_link2 = -(rotation_axis_wrt_link1).getRotationTwist(1.0);
    this->S_link2_link1 = (link1_X_link2_at_rest.inverse()*rotation_axis_wrt_link1).getRotationTwist(1.0);
}

Transform RevoluteJoint::getTransform(const VectorDynSize& jntPos, const LinkIndex p_linkA, const LinkIndex p_linkB) const
{
    const double ang = jntPos(this->getPosCoordsOffset());
    updateBuffers(ang);
    if( p_linkA == link1 )
    {
        assert(p_linkB == link2);
        return this->link1_X_link2;
    }
    else
    {
        assert(p_linkA == link2);
        assert(p_linkB == link1);
        return this->link2_X_link1;
    }
}

SpatialMotionVector RevoluteJoint::getMotionSubspaceVector(int dof_i,
                                                           const LinkIndex p_linkA,
                                                           const LinkIndex p_linkB) const
{
    if( p_linkA == link2 )
    {
        return this->S_link2_link1;
    }
    else
    {
        return this->S_link1_link2;
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

    this->resetAxisBuffers();
}

void RevoluteJoint::computeChildVelAcc(const VectorDynSize & jntPos,
                                       const VectorDynSize & jntVel,
                                       const VectorDynSize & jntAcc,
                                       LinkVelArray & linkVels,
                                       LinkAccArray & linkAccs,
                                       const LinkIndex child, const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());
    double d2ang = jntAcc(this->getDOFsOffset());

    Transform child_X_parent = this->getTransform(jntPos,child,parent);

    // Propagate twist and spatial acceleration: for a revolute joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    Twist vJ_link1 = rotation_axis_wrt_link1.getRotationTwist(dang);
    SpatialAcc aJ_link1 = rotation_axis_wrt_link1.getRotationSpatialAcc(d2ang);

    if( parent == link1 )
    {
        linkVels(child) = child_X_parent*(linkVels(parent) + vJ_link1);
        linkAccs(child) = child_X_parent*(linkAccs(parent) + aJ_link1) + linkVels(child)*(child_X_parent*vJ_link1);
    }
    else
    {
        linkVels(child) = child_X_parent*linkVels(parent)  - vJ_link1;
        linkAccs(child) = child_X_parent*linkAccs(parent) - aJ_link1 - linkVels(child)*vJ_link1;
    }

    return;
}

void RevoluteJoint::computeChildPosVelAcc(const VectorDynSize & jntPos,
                                          const VectorDynSize & jntVel,
                                          const VectorDynSize & jntAcc,
                                          LinkPositions & linkPositions,
                                          LinkVelArray & linkVels,
                                          LinkAccArray & linkAccs,
                                          const LinkIndex child,
                                          const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());
    double d2ang = jntAcc(this->getDOFsOffset());

    Transform child_X_parent = this->getTransform(jntPos,child,parent);
    Transform parent_X_child = this->getTransform(jntPos,parent,child);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_H_child  = ref_H_parent*parent_H_child
    linkPositions(child) = linkPositions(parent)*parent_X_child;

    // Propagate twist and spatial acceleration: for a revolute joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    Twist vJ_link1 = rotation_axis_wrt_link1.getRotationTwist(dang);
    SpatialAcc aJ_link1 = rotation_axis_wrt_link1.getRotationSpatialAcc(d2ang);

    if( parent == link1 )
    {
        linkVels(child) = child_X_parent*(linkVels(parent) + vJ_link1);
        linkAccs(child) = child_X_parent*(linkAccs(parent) + aJ_link1) + linkVels(child)*(child_X_parent*vJ_link1);
    }
    else
    {
        linkVels(child) = child_X_parent*linkVels(parent)  - vJ_link1;
        linkAccs(child) = child_X_parent*linkAccs(parent) - aJ_link1 - linkVels(child)*vJ_link1;
    }

    return;
}


void RevoluteJoint::computeJointTorque(const VectorDynSize& jntPos, const Wrench& internalWrench,
                                       LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied,
                                       VectorDynSize& jntTorques) const
{
    double & tau = jntTorques(this->getDOFsOffset());

    if( linkOnWhichWrenchIsApplied == link2 )
    {
        // in this case link2 is the child and link1 is the parent
        iDynTree::Twist S_wrt_link2 = this->getMotionSubspaceVector(0,link2,link1);
        tau = S_wrt_link2.dot(internalWrench);
    }
    else
    {
         // in this case link1 is the child and link2 is the parent
        iDynTree::Twist S_wrt_link1 = this->getMotionSubspaceVector(0,link1,link2);
        tau = S_wrt_link1.dot(internalWrench);
    }

}







}
