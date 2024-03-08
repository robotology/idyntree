// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/Utils.h>

#include <iDynTree/FixedJoint.h>
#include <iDynTree/LinkState.h>

#include <cassert>

namespace iDynTree
{

FixedJoint::FixedJoint()
: FixedJoint(Transform::Identity()) {}

FixedJoint::FixedJoint(const Transform& _link1_X_link2)
: FixedJoint(LINK_INVALID_INDEX, LINK_INVALID_INDEX, _link1_X_link2)
{
}


FixedJoint::FixedJoint(const LinkIndex _link1, const LinkIndex _link2,
                       const Transform& _link1_X_link2):
                       link1(_link1), link2(_link2),
                       link1_X_link2(_link1_X_link2),
                       link2_X_link1(_link1_X_link2.inverse())
{
}

FixedJoint::FixedJoint(const FixedJoint& other):
                       link1(other.link1), link2(other.link2),
                       link1_X_link2(other.link1_X_link2),
                       link2_X_link1(other.link2_X_link1)
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
    link2_X_link1 = _link1_X_link2.inverse();
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

Transform FixedJoint::getRestTransform(const LinkIndex child, const LinkIndex parent) const
{
    if( child == this->link1 )
    {
        return this->link1_X_link2;
    }
    else
    {
        assert(child == this->link2);
        assert(parent == this->link1);
        return this->link2_X_link1;
    }
}

const Transform & FixedJoint::getTransform(const VectorDynSize & jntPos, const LinkIndex child, const LinkIndex parent) const
{
    if( child == this->link1 )
    {
        return this->link1_X_link2;
    }
    else
    {
        assert(child == this->link2);
        assert(parent == this->link1);
        return this->link2_X_link1;
    }
}

TransformDerivative FixedJoint::getTransformDerivative(const VectorDynSize& jntPos, const LinkIndex child, const LinkIndex parent, const int posCoord_i) const
{
    return TransformDerivative::Zero();
}


SpatialMotionVector FixedJoint::getMotionSubspaceVector(int dof_i, const LinkIndex child, const LinkIndex parent) const
{
    return SpatialMotionVector::Zero();
}

void FixedJoint::computeJointTorque(const VectorDynSize & jntPos, const Wrench& internalWrench,
                                    LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied,
                                    VectorDynSize& jntTorques) const
{
    // A fixed joint would have a torque of size 0
    return;
}

void FixedJoint::computeChildPosVelAcc(const VectorDynSize & jntPos,
                                       const VectorDynSize & jntVel,
                                       const VectorDynSize & jntAcc,
                                           LinkPositions & linkPositions,
                                           LinkVelArray & linkVels,
                                           LinkAccArray & linkAccs,
                                           const LinkIndex child,
                                           const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    const Transform & parent_X_child = this->getTransform(jntPos,parent,child);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_H_child  = ref_H_parent*parent_H_child
    linkPositions(child) = linkPositions(parent)*parent_X_child;

    // Propagate twist and spatial acceleration: for a fixed joint the twist of two attached links is the same,
    // expect that they are usually expressed in different frames
    linkVels(child) = child_X_parent*linkVels(parent);
    linkAccs(child) = child_X_parent*linkAccs(parent);

    return;
}

void FixedJoint::computeChildVelAcc(const VectorDynSize & jntPos,
                                    const VectorDynSize & jntVel,
                                    const VectorDynSize & jntAcc,
                                          LinkVelArray & linkVels,
                                          LinkAccArray & linkAccs,
                                          const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    // Propagate twist: for a fixed joint the twist of two attached links is the same,
    // expect that they are usually expressed in different frames
    linkVels(child) = child_X_parent*linkVels(parent);
    linkAccs(child) = child_X_parent*linkAccs(parent);

    return;
}

void FixedJoint::computeChildVel(const VectorDynSize & jntPos,
                                 const VectorDynSize & jntVel,
                                       LinkVelArray & linkVels,
                                       const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    // Propagate twist: for a fixed joint the twist of two attached links is the same,
    // expect that they are usually expressed in different frames
    linkVels(child) = child_X_parent*linkVels(parent);
    return;
}

void FixedJoint::computeChildAcc(const VectorDynSize &jntPos, const VectorDynSize &jntVel,
                                    const LinkVelArray &linkVels, const VectorDynSize &jntAcc,
                                    LinkAccArray &linkAccs, const LinkIndex child, const LinkIndex parent) const
{

    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    linkAccs(child) = child_X_parent*linkAccs(parent);
}

void FixedJoint::computeChildBiasAcc(const VectorDynSize &jntPos,
                                        const VectorDynSize &jntVel,
                                        const LinkVelArray &linkVels,
                                        LinkAccArray &linkBiasAccs,
                                        const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    linkBiasAccs(child) = child_X_parent*linkBiasAccs(parent);
}

void FixedJoint::setIndex(JointIndex& _index)
{
    this->m_index = _index;
}

JointIndex FixedJoint::getIndex() const
{
    return this->m_index;
}

void FixedJoint::setPosCoordsOffset(const size_t _offset)
{
    this->m_posCoordsOffset = _offset;
}

size_t FixedJoint::getPosCoordsOffset() const
{
    return this->m_posCoordsOffset;
}

void FixedJoint::setDOFsOffset(const size_t _offset)
{
    this->m_DOFsOffset = _offset;
}

size_t FixedJoint::getDOFsOffset() const
{
    return this->m_DOFsOffset;
}

bool FixedJoint::hasPosLimits() const
{
    return false;
}

bool FixedJoint::enablePosLimits(const bool /*enable*/)
{
    return false;
}

bool FixedJoint::getPosLimits(const size_t _index, double & min, double & max) const
{
    return false;
}

double FixedJoint::getMinPosLimit(const size_t _index) const
{
    return 0.0;
}

double FixedJoint::getMaxPosLimit(const size_t _index) const
{
    return 0.0;
}

bool FixedJoint::setPosLimits(const size_t /*_index*/, double /*min*/, double /*max*/)
{
    return false;
}

JointDynamicsType FixedJoint::getJointDynamicsType() const
{
    return NoJointDynamics;
}

bool FixedJoint::setJointDynamicsType(const JointDynamicsType enable)
{
    return false;
}

double FixedJoint::getDamping(const size_t _index) const
{
    return 0.0;
}
double FixedJoint::getStaticFriction(const size_t _index) const
{
    return 0.0;
}

bool FixedJoint::setDamping(const size_t /*_index*/, double /*damping*/)
{
    return false;
}

bool FixedJoint::setStaticFriction(const size_t /*_index*/, double /*staticFriction*/)
{
    return false;
}

}
