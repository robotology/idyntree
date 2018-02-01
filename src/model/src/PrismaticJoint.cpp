/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Model/PrismaticJoint.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/TransformDerivative.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>

#include <cassert>

#include <cfloat>

namespace iDynTree
{

PrismaticJoint::PrismaticJoint():
        link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX), link1_X_link2_at_rest(Transform::Identity()),
        translation_axis_wrt_link1(Axis(Direction(1.0, 0.0, 0.0), Position::Zero()))
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(0);
    this->disablePosLimits();
}

PrismaticJoint::PrismaticJoint(const LinkIndex _link1, const LinkIndex _link2,
                             const Transform& _link1_X_link2, const Axis& _translation_axis_wrt_link1):
                             link1(_link1), link2(_link2), link1_X_link2_at_rest(_link1_X_link2),
                             translation_axis_wrt_link1(_translation_axis_wrt_link1)
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(0);
    this->disablePosLimits();
}

PrismaticJoint::PrismaticJoint(const PrismaticJoint& other):
                             link1(other.link1), link2(other.link2),
                             link1_X_link2_at_rest(other.link1_X_link2_at_rest),
                             translation_axis_wrt_link1(other.translation_axis_wrt_link1),
                             m_hasPosLimits(other.m_hasPosLimits),
                             m_minPos(other.m_minPos), m_maxPos(other.m_maxPos)
{
    this->setPosCoordsOffset(other.getPosCoordsOffset());
    this->setDOFsOffset(other.getDOFsOffset());

    this->resetAxisBuffers();
    this->resetBuffers(0);
}

PrismaticJoint::~PrismaticJoint()
{
}

IJoint* PrismaticJoint::clone() const
{
    return (IJoint *) new PrismaticJoint(*this);
}


void PrismaticJoint::setRestTransform(const Transform& _link1_X_link2)
{
    this->link1_X_link2_at_rest = _link1_X_link2;

    this->resetAxisBuffers();
}

LinkIndex PrismaticJoint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex PrismaticJoint::getSecondAttachedLink() const
{
    return link2;
}

Transform PrismaticJoint::getRestTransform(const LinkIndex p_linkA, const LinkIndex p_linkB) const
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

void PrismaticJoint::updateBuffers(const double new_q) const
{
    if( new_q != q_previous )
    {
        resetBuffers(new_q);
    }

    return;
}

void PrismaticJoint::resetBuffers(const double new_q) const
{
    this->link1_X_link2 = translation_axis_wrt_link1.getTranslationTransform(new_q)*link1_X_link2_at_rest;
    this->link2_X_link1 = link1_X_link2.inverse();

    q_previous = new_q;
}

void PrismaticJoint::resetAxisBuffers() const
{
    this->S_link1_link2 = -translation_axis_wrt_link1.getTranslationTwist(1.0);
    this->S_link2_link1 = (link1_X_link2_at_rest.inverse()*translation_axis_wrt_link1).getTranslationTwist(1.0);
}

const Transform & PrismaticJoint::getTransform(const VectorDynSize& jntPos,
                                              const LinkIndex p_linkA,
                                              const LinkIndex p_linkB) const
{
    const double dist = jntPos(this->getPosCoordsOffset());
    updateBuffers(dist);
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

TransformDerivative PrismaticJoint::getTransformDerivative(const VectorDynSize& jntPos,
                                                          const LinkIndex linkA,
                                                          const LinkIndex linkB,
                                                          const int posCoord_i) const
{
    const double dist = jntPos(this->getPosCoordsOffset());

    TransformDerivative link1_dX_link2 = translation_axis_wrt_link1.getTranslationTransformDerivative(dist)*link1_X_link2_at_rest;

    if( linkA == this->link1 )
    {
        return link1_dX_link2;
    }
    else
    {
        updateBuffers(dist);
        TransformDerivative linkA_dX_linkB = link1_dX_link2.derivativeOfInverse(this->link1_X_link2);
        return linkA_dX_linkB;
    }
}


SpatialMotionVector PrismaticJoint::getMotionSubspaceVector(int dof_i,
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


Axis PrismaticJoint::getAxis(const LinkIndex child,
                             const LinkIndex /*parent*/) const
{
    if( child == link1 )
    {
        return translation_axis_wrt_link1.reverse();
    }
    else
    {
        assert(child == link2);
        return link1_X_link2_at_rest.inverse()*translation_axis_wrt_link1;
    }
}

void PrismaticJoint::setAttachedLinks(const LinkIndex _link1, const LinkIndex _link2)
{
    this->link1 = _link1;
    this->link2 = _link2;
}

void PrismaticJoint::setAxis(const Axis& prismaticAxis_wrt_link1)
{
    this->translation_axis_wrt_link1 = prismaticAxis_wrt_link1;

    this->resetAxisBuffers();
}

void PrismaticJoint::setAxis(const Axis &prismaticAxis,
                             const LinkIndex child, const LinkIndex /*parent*/)
{
    if( child == link1 )
    {
        translation_axis_wrt_link1 = prismaticAxis.reverse();
    }
    else
    {
        assert(child == link2);
        translation_axis_wrt_link1 = link1_X_link2_at_rest*prismaticAxis;
    }
}

void PrismaticJoint::computeChildVelAcc(const VectorDynSize & jntPos,
                                       const VectorDynSize & jntVel,
                                       const VectorDynSize & jntAcc,
                                       LinkVelArray & linkVels,
                                       LinkAccArray & linkAccs,
                                       const LinkIndex child, const LinkIndex parent) const
{
    double ddist = jntVel(this->getDOFsOffset());
    double d2dist = jntAcc(this->getDOFsOffset());

    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);

    // Propagate twist and spatial acceleration: for a prismatic joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);

    SpatialMotionVector vj = S*ddist;
    linkVels(child) = child_X_parent*linkVels(parent) + vj;
    linkAccs(child) = child_X_parent*linkAccs(parent) + S*d2dist + linkVels(child)*vj;

    return;
}


void PrismaticJoint::computeChildVel(const VectorDynSize & jntPos,
                                    const VectorDynSize & jntVel,
                                          LinkVelArray & linkVels,
                                    const LinkIndex child, const LinkIndex parent) const
{
    double ddist = jntVel(this->getDOFsOffset());

    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);

    // Propagate twist and spatial acceleration: for a prismatic joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);

    SpatialMotionVector vj = S*ddist;
    linkVels(child) = child_X_parent*linkVels(parent) + vj;

    return;
}

void PrismaticJoint::computeChildPosVelAcc(const VectorDynSize & jntPos,
                                          const VectorDynSize & jntVel,
                                          const VectorDynSize & jntAcc,
                                          LinkPositions & linkPositions,
                                          LinkVelArray & linkVels,
                                          LinkAccArray & linkAccs,
                                          const LinkIndex child,
                                          const LinkIndex parent) const
{
    double ddist = jntVel(this->getDOFsOffset());
    double d2dist = jntAcc(this->getDOFsOffset());

    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    const Transform & parent_X_child = this->getTransform(jntPos,parent,child);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_H_child  = ref_H_parent*parent_H_child
    linkPositions(child) = linkPositions(parent)*parent_X_child;

    // Propagate twist and spatial acceleration: for a prismatic joint (as for any 1 dof joint)
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);

    SpatialMotionVector vj = S*ddist;
    linkVels(child) = child_X_parent*linkVels(parent) + vj;
    linkAccs(child) = child_X_parent*linkAccs(parent) + S*d2dist + linkVels(child)*vj;

    return;
}

void PrismaticJoint::computeChildAcc(const VectorDynSize &jntPos, const VectorDynSize &jntVel,
                                    const LinkVelArray &linkVels, const VectorDynSize &jntAcc,
                                    LinkAccArray &linkAccs, const LinkIndex child, const LinkIndex parent) const
{
    double ddist = jntVel(this->getDOFsOffset());
    double d2dist = jntAcc(this->getDOFsOffset());
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    const Transform & parent_X_child = this->getTransform(jntPos,parent,child);
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);
    SpatialMotionVector vj = S*ddist;
    linkAccs(child) = child_X_parent*linkAccs(parent) + S*d2dist + linkVels(child)*vj;
}

void PrismaticJoint::computeChildBiasAcc(const VectorDynSize &jntPos,
                                        const VectorDynSize &jntVel,
                                        const LinkVelArray &linkVels,
                                              LinkAccArray &linkBiasAccs,
                                         const LinkIndex child, const LinkIndex parent) const
{
    double ddist = jntVel(this->getDOFsOffset());
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    const Transform & parent_X_child = this->getTransform(jntPos,parent,child);
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);
    SpatialMotionVector vj = S*ddist;
    linkBiasAccs(child) = child_X_parent*linkBiasAccs(parent) + linkVels(child)*vj;
}

void PrismaticJoint::computeJointTorque(const VectorDynSize& jntPos, const Wrench& internalWrench,
                                       LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied,
                                       VectorDynSize& jntTorques) const
{
    double & tau = jntTorques(this->getDOFsOffset());

    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,linkOnWhichWrenchIsApplied);
    tau = S.dot(internalWrench);

    return;
}

void PrismaticJoint::disablePosLimits()
{
    m_hasPosLimits = false;
    m_minPos = -DBL_MAX;
    m_maxPos = DBL_MAX;
}

bool PrismaticJoint::hasPosLimits() const
{
    return m_hasPosLimits;
}

bool PrismaticJoint::enablePosLimits(const bool enable)
{
    m_hasPosLimits = enable;
    return true;
}

bool PrismaticJoint::getPosLimits(const size_t /*_index*/, double & min, double & max) const
{
    min = m_minPos;
    max = m_maxPos;

    return true;
}

double PrismaticJoint::getMinPosLimit(const size_t /*_index*/) const
{
    return m_minPos;
}

double PrismaticJoint::getMaxPosLimit(const size_t /*_index*/) const
{
    return m_maxPos;
}

bool PrismaticJoint::setPosLimits(const size_t /*_index*/, double & min, double & max)
{
    m_minPos = min;
    m_maxPos = max;

    return true;
}



}
