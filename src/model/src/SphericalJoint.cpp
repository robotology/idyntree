// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SphericalJoint.h>

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/Twist.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/GeomVector3.h>
#include <iDynTree/Span.h>
#include <iDynTree/MatrixView.h>
#include <iDynTree/Direction.h>            // added
#include <iDynTree/MovableJointImpl.h>     // added
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixFixSize.h>

#include <cassert>
#include <cmath>

namespace iDynTree
{

SphericalJoint::SphericalJoint():
        link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX),
        link1_X_link2_at_rest(Transform::Identity()), m_center_wrt_link1(Position::Zero())
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    // Initialize with identity quaternion (w,x,y,z) = (1,0,0,0)
    Vector4 identityQuat;
    identityQuat(0) = 1.0; identityQuat(1) = 0.0; identityQuat(2) = 0.0; identityQuat(3) = 0.0;
    this->resetAxisBuffers();
    this->resetBuffers(identityQuat);
}

SphericalJoint::SphericalJoint(const Transform& _link1_X_link2):
                             link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX),
                             link1_X_link2_at_rest(_link1_X_link2), m_center_wrt_link1(Position::Zero())
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    // Initialize with identity quaternion (w,x,y,z) = (1,0,0,0)
    Vector4 identityQuat;
    identityQuat(0) = 1.0; identityQuat(1) = 0.0; identityQuat(2) = 0.0; identityQuat(3) = 0.0;
    this->resetAxisBuffers();
    this->resetBuffers(identityQuat);
}

SphericalJoint::SphericalJoint(const SphericalJoint& other):
                             link1(other.link1), link2(other.link2),
                             link1_X_link2_at_rest(other.link1_X_link2_at_rest), m_center_wrt_link1(other.m_center_wrt_link1)
{
    this->setPosCoordsOffset(other.getPosCoordsOffset());
    this->setDOFsOffset(other.getDOFsOffset());

    this->resetAxisBuffers();
    this->resetBuffers(other.q_previous);
}

SphericalJoint::~SphericalJoint()
{
}

IJoint* SphericalJoint::clone() const
{
    return (IJoint *) new SphericalJoint(*this);
}

void SphericalJoint::setAttachedLinks(const LinkIndex _link1, const LinkIndex _link2)
{
    this->link1 = _link1;
    this->link2 = _link2;
}

void SphericalJoint::setRestTransform(const Transform& _link1_X_link2)
{
    this->link1_X_link2_at_rest = _link1_X_link2;

    // Changing rest transform affects cached transforms
    this->resetAxisBuffers();
}

void SphericalJoint::setJointCenter(const LinkIndex link, const Position& center)
{
    if (link == this->link1)
    {
        m_center_wrt_link1 = center;
    }
    else if (link == this->link2)
    {
        // Transform center from link2 frame to link1 frame and store
        m_center_wrt_link1 = this->link1_X_link2_at_rest * center;
    }
}

Position SphericalJoint::getJointCenter(const LinkIndex link) const
{
    if (link == this->link1)
    {
        return m_center_wrt_link1;
    }
    else if (link == this->link2)
    {
        return  this->link1_X_link2_at_rest.inverse() * m_center_wrt_link1;
    }
    return Position::Zero();
}

LinkIndex SphericalJoint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex SphericalJoint::getSecondAttachedLink() const
{
    return link2;
}

void SphericalJoint::normalizeQuaternion(Vector4& q) const
{
    // Normalize quaternion (w,x,y,z) where w is real part, (x,y,z) is imaginary part
    double norm = std::sqrt(q(0)*q(0) + q(1)*q(1) + q(2)*q(2) + q(3)*q(3));
    if (norm > 1e-12)
    {
        q(0) /= norm; q(1) /= norm; q(2) /= norm; q(3) /= norm;
    }
    else
    {
        // Fallback to identity if degenerate
        q(0) = 1.0; q(1) = 0.0; q(2) = 0.0; q(3) = 0.0;
    }
}

void SphericalJoint::resetBuffers(const Vector4& new_q) const
{
    this->q_previous = new_q;

    // Current rotation: R_cur = R_rest * R_delta
    Rotation R_rest = this->link1_X_link2_at_rest.getRotation();
    Rotation R_delta;
    R_delta.fromQuaternion(new_q);
    Rotation R_cur = R_rest * R_delta;

    // p = r1 - R_cur r2 for {}^1 H_2
    Position centerInSecondLink = getJointCenter(this->link2);
    Position p_cur;
    toEigen(p_cur) = toEigen(m_center_wrt_link1) - toEigen(R_cur) * toEigen(centerInSecondLink);

    this->link1_X_link2 = Transform(R_cur, p_cur);
    this->link2_X_link1 = this->link1_X_link2.inverse();
}

void SphericalJoint::updateBuffers(const Vector4& new_q) const
{
   // Check if quaternion has changed (manual comparison since Vector4 doesn't have != operator)
    bool quaternionChanged = false;
    for (int i = 0; i < new_q.size(); i++)
    {
        if (std::abs(new_q(i) - q_previous(i)) > 1e-14)
        {
            quaternionChanged = true;
            break;
        }
    }

    if (quaternionChanged)
    {
        resetBuffers(new_q);
    }
}

void SphericalJoint::resetAxisBuffers() const
{
    LinearMotionVector3 v_lin; v_lin.zero();
    AngularMotionVector3 wx, wy, wz;
    wx(0) = 1.0; wx(1) = 0.0; wx(2) = 0.0;
    wy(0) = 0.0; wy(1) = 1.0; wy(2) = 0.0;
    wz(0) = 0.0; wz(1) = 0.0; wz(2) = 1.0;

    this->S_link1_link2[0] = SpatialMotionVector(v_lin, wx);
    this->S_link1_link2[1] = SpatialMotionVector(v_lin, wy);
    this->S_link1_link2[2] = SpatialMotionVector(v_lin, wz);

    this->S_link2_link1[0] = SpatialMotionVector(v_lin, wx);
    this->S_link2_link1[1] = SpatialMotionVector(v_lin, wy);
    this->S_link2_link1[2] = SpatialMotionVector(v_lin, wz);
}

Transform SphericalJoint::getRestTransform(const LinkIndex child, const LinkIndex parent) const
{
    if( child == link1 )
    {
        assert(parent == link2);
        return link1_X_link2_at_rest;
    }
    else
    {
        assert(child == link2);
        assert(parent == link1);
        return link1_X_link2_at_rest.inverse();
    }
}

const Transform & SphericalJoint::getTransform(const VectorDynSize & jntPos, const LinkIndex child, const LinkIndex parent) const
{
    // Extract quaternion from joint position vector
    Vector4 q;
    for (int i = 0; i < 4; i++)
    {
        q(i) = jntPos(this->getPosCoordsOffset() + i);
    }

    // Normalize quaternion and update center-aware buffers
    normalizeQuaternion(q);
    this->updateBuffers(q);

    // Return according to requested direction
    if (child == this->link1)
    {
        return this->link1_X_link2;
    }
    else
    {
        return this->link2_X_link1;
    }
}

TransformDerivative SphericalJoint::getTransformDerivative(const VectorDynSize & jntPos,
                                                          const LinkIndex child,
                                                          const LinkIndex parent,
                                                          const int posCoord_i) const
{
    // Central finite difference remains valid (quaternion renormalized), center-aware via getTransform
    const double h = 1e-8;
    const size_t off = this->getPosCoordsOffset();
    const size_t idx = off + static_cast<size_t>(posCoord_i);

    VectorDynSize posPlus = jntPos;
    VectorDynSize posMinus = jntPos;

    posPlus(idx) += h;
    posMinus(idx) -= h;

    auto normalizeQuatInPos = [&](VectorDynSize& v)
    {
        double w = v(off + 0);
        double x = v(off + 1);
        double y = v(off + 2);
        double z = v(off + 3);
        double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 1e-12) {
            v(off + 0) = w/n;
            v(off + 1) = x/n;
            v(off + 2) = y/n;
            v(off + 3) = z/n;
        } else {
            v(off + 0) = 1.0; v(off + 1) = 0.0; v(off + 2) = 0.0; v(off + 3) = 0.0;
        }
    };

    normalizeQuatInPos(posPlus);
    normalizeQuatInPos(posMinus);

    Transform Hplus  = this->getTransform(posPlus,  child, parent);
    Transform Hminus = this->getTransform(posMinus, child, parent);

    Matrix4x4 dH;
    toEigen(dH) = (toEigen(Hplus.asHomogeneousTransform()) - toEigen(Hminus.asHomogeneousTransform())) / (2.0*h);

    TransformDerivative out;
    out.fromHomogeneousTransformDerivative(dH);
    return out;
}

SpatialMotionVector SphericalJoint::getMotionSubspaceVector(int dof_i,
                                                           const LinkIndex child,
                                                           const LinkIndex parent) const
{
    assert(dof_i >= 0 && dof_i < 3);

    // Unit basis vectors for angular part
    AngularMotionVector3 ex, ey, ez;
    ex(0) = 1.0; ex(1) = 0.0; ex(2) = 0.0;
    ey(0) = 0.0; ey(1) = 1.0; ey(2) = 0.0;
    ez(0) = 0.0; ez(1) = 0.0; ez(2) = 1.0;

    if( child == this->link2 && parent == this->link1 )
    {
        // Motion subspace expressed in child (link2) frame
        // Get center position in child (link2) frame
        const Position centerInSecondLink = getJointCenter(this->link2);

        AngularMotionVector3 omega;
        LinearMotionVector3 v_lin;

        if (dof_i == 0) {
            omega = ex;
        } else if (dof_i == 1) {
            omega = ey;
        } else {
            omega = ez;
        }

        // v = ω × (-r_center) = -ω × r_center
        v_lin(0) = omega(1) * (-centerInSecondLink(2)) - omega(2) * (-centerInSecondLink(1));
        v_lin(1) = omega(2) * (-centerInSecondLink(0)) - omega(0) * (-centerInSecondLink(2));
        v_lin(2) = omega(0) * (-centerInSecondLink(1)) - omega(1) * (-centerInSecondLink(0));

        return SpatialMotionVector(v_lin, omega);
    }
    else if( child == this->link1 && parent == this->link2 )
    {
        // Motion subspace expressed in child (link1) frame
        const Rotation& R1_2 = this->link1_X_link2.getRotation();

        // Get center position in child (link1) frame
        const Position& centerInFirstLink = getJointCenter(this->link1);

        AngularMotionVector3 omega_child;
        LinearMotionVector3 v_lin;

        if (dof_i == 0) {
            Direction e2(1.0, 0.0, 0.0);
            Direction mapped = R1_2*e2;
            omega_child(0) = -mapped(0); omega_child(1) = -mapped(1); omega_child(2) = -mapped(2);
        } else if (dof_i == 1) {
            Direction e2(0.0, 1.0, 0.0);
            Direction mapped = R1_2*e2;
            omega_child(0) = -mapped(0); omega_child(1) = -mapped(1); omega_child(2) = -mapped(2);
        } else {
            Direction e2(0.0, 0.0, 1.0);
            Direction mapped = R1_2*e2;
            omega_child(0) = -mapped(0); omega_child(1) = -mapped(1); omega_child(2) = -mapped(2);
        }

        // Linear velocity due to rotation about center: v = ω × (-r_center)
        v_lin(0) = omega_child(1) * (-centerInFirstLink(2)) - omega_child(2) * (-centerInFirstLink(1));
        v_lin(1) = omega_child(2) * (-centerInFirstLink(0)) - omega_child(0) * (-centerInFirstLink(2));
        v_lin(2) = omega_child(0) * (-centerInFirstLink(1)) - omega_child(1) * (-centerInFirstLink(0));

        return SpatialMotionVector(v_lin, omega_child);
    }
    else
    {
        return SpatialMotionVector::Zero();
    }
}

void SphericalJoint::computeChildPosVelAcc(const VectorDynSize & jntPos,
                                          const VectorDynSize & jntVel,
                                          const VectorDynSize & jntAcc,
                                          LinkPositions & linkPositions,
                                          LinkVelArray & linkVels,
                                          LinkAccArray & linkAccs,
                                          const LinkIndex child,
                                          const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos, child, parent);
    const Transform & parent_X_child = this->getTransform(jntPos, parent, child);

    // Propagate position
    linkPositions(child) = linkPositions(parent) * parent_X_child;

    // Propagate velocity: for spherical joint we need to add joint velocity contribution
    Twist joint_twist = Twist::Zero();
    for (int dof = 0; dof < 3; dof++)
    {
        joint_twist = joint_twist + this->getMotionSubspaceVector(dof, child, parent) * jntVel(this->getDOFsOffset() + dof);
    }
    linkVels(child) = child_X_parent * linkVels(parent) + joint_twist;

    // Propagate acceleration: similar to velocity but also including joint acceleration
    SpatialAcc joint_acc = SpatialAcc::Zero();
    for (int dof = 0; dof < 3; dof++)
    {
        joint_acc = joint_acc + this->getMotionSubspaceVector(dof, child, parent) * jntAcc(this->getDOFsOffset() + dof);
    }

    linkAccs(child) = child_X_parent * linkAccs(parent) + joint_acc;
}

void SphericalJoint::computeChildVel(const VectorDynSize & jntPos,
                                    const VectorDynSize & jntVel,
                                    LinkVelArray & linkVels,
                                    const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos, child, parent);

    // Propagate velocity with joint velocity contribution
    Twist joint_twist = Twist::Zero();
    for (int dof = 0; dof < 3; dof++)
    {
        joint_twist = joint_twist + this->getMotionSubspaceVector(dof, child, parent) * jntVel(this->getDOFsOffset() + dof);
    }
    linkVels(child) = child_X_parent * linkVels(parent) + joint_twist;
}

void SphericalJoint::computeChildVelAcc(const VectorDynSize & jntPos,
                                       const VectorDynSize & jntVel,
                                       const VectorDynSize & jntAcc,
                                       LinkVelArray & linkVels,
                                       LinkAccArray & linkAccs,
                                       const LinkIndex child, const LinkIndex parent) const
{
    // Compute velocity first
    this->computeChildVel(jntPos, jntVel, linkVels, child, parent);

    // Then compute acceleration
    const Transform & child_X_parent = this->getTransform(jntPos, child, parent);

    SpatialAcc joint_acc = SpatialAcc::Zero();
    for (int dof = 0; dof < 3; dof++)
    {
        joint_acc = joint_acc + this->getMotionSubspaceVector(dof, child, parent) * jntAcc(this->getDOFsOffset() + dof);
    }

    linkAccs(child) = child_X_parent * linkAccs(parent) + joint_acc;
}

void SphericalJoint::computeChildAcc(const VectorDynSize & jntPos,
                                    const VectorDynSize & jntVel,
                                    const LinkVelArray & linkVels,
                                    const VectorDynSize & jntAcc,
                                    LinkAccArray & linkAccs,
                                    const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos, child, parent);

    SpatialAcc joint_acc = SpatialAcc::Zero();
    for (int dof = 0; dof < 3; dof++)
    {
        joint_acc = joint_acc + this->getMotionSubspaceVector(dof, child, parent) * jntAcc(this->getDOFsOffset() + dof);
    }

    linkAccs(child) = child_X_parent * linkAccs(parent) + joint_acc;
}

void SphericalJoint::computeChildBiasAcc(const VectorDynSize & jntPos,
                                        const VectorDynSize & jntVel,
                                        const LinkVelArray & linkVels,
                                        LinkAccArray & linkBiasAccs,
                                        const LinkIndex child, const LinkIndex parent) const
{
    const Transform & child_X_parent = this->getTransform(jntPos, child, parent);

    linkBiasAccs(child) = child_X_parent * linkBiasAccs(parent);
}

void SphericalJoint::computeJointTorque(const VectorDynSize & jntPos,
                                       const Wrench & internalWrench,
                                       const LinkIndex linkThatAppliesWrench,
                                       const LinkIndex linkOnWhichWrenchIsApplied,
                                       VectorDynSize & jntTorques) const
{
    // Project the wrench onto the motion subspace vectors
    for (int dof = 0; dof < 3; dof++)
    {
        SpatialMotionVector S = this->getMotionSubspaceVector(dof, linkOnWhichWrenchIsApplied, linkThatAppliesWrench);
        jntTorques(this->getDOFsOffset() + dof) = internalWrench.dot(S);
    }
}

// Joint limits and dynamics are not supported for spherical joints
bool SphericalJoint::hasPosLimits() const
{
    return false;
}

bool SphericalJoint::enablePosLimits(const bool /*enable*/)
{
    return false;
}

bool SphericalJoint::getPosLimits(const size_t /*_index*/, double & /*min*/, double & /*max*/) const
{
    return false;
}

double SphericalJoint::getMinPosLimit(const size_t /*_index*/) const
{
    return 0.0;
}

double SphericalJoint::getMaxPosLimit(const size_t /*_index*/) const
{
    return 0.0;
}

bool SphericalJoint::setPosLimits(const size_t /*_index*/, double /*min*/, double /*max*/)
{
    return false;
}

JointDynamicsType SphericalJoint::getJointDynamicsType() const
{
    // No damping/friction by default
    return NoJointDynamics;
}

bool SphericalJoint::setJointDynamicsType(const JointDynamicsType /*enable*/)
{
    return false;
}

bool SphericalJoint::setDamping(const size_t /*_index*/, double /*damping*/)
{
    return false;
}

bool SphericalJoint::setStaticFriction(const size_t /*_index*/, double /*staticFriction*/)
{
    return false;
}

double SphericalJoint::getDamping(const size_t /*_index*/) const
{
    return 0.0;
}

double SphericalJoint::getStaticFriction(const size_t /*_index*/) const
{
    return 0.0;
}

bool SphericalJoint::getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos,
                                                          iDynTree::MatrixView<double>& jac) const
{
    // Expect 4x3 view
    if (jac.rows() != 4 || jac.cols() != 3) {
        return false;
    }

    const size_t off = this->getPosCoordsOffset();
    double w = jntPos[off + 0];
    double x = jntPos[off + 1];
    double y = jntPos[off + 2];
    double z = jntPos[off + 3];

    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 1e-12) { w/=n; x/=n; y/=n; z/=n; } else { w=1.0; x=y=z=0.0; }

    // q̇ = 0.5 * [ -v^T ; w I + [v]_x ] ω
    jac(0,0) = -0.5 * x;
    jac(0,1) = -0.5 * y;
    jac(0,2) = -0.5 * z;

    jac(1,0) = 0.5 * ( w );
    jac(1,1) = 0.5 * ( - z );
    jac(1,2) = 0.5 * ( y );

    jac(2,0) = 0.5 * ( z );
    jac(2,1) = 0.5 * ( w );
    jac(2,2) = 0.5 * ( -x );

    jac(3,0) = 0.5 * ( -y );
    jac(3,1) = 0.5 * ( x );
    jac(3,2) = 0.5 * ( w );

    return true;
}

bool SphericalJoint::setJointPosCoordsToRest(iDynTree::Span<double> jntPos) const
{
    if (jntPos.size() < this->getPosCoordsOffset()+4) return false;
    jntPos[this->getPosCoordsOffset()+0] = 1.0;
    jntPos[this->getPosCoordsOffset()+1] = 0.0;
    jntPos[this->getPosCoordsOffset()+2] = 0.0;
    jntPos[this->getPosCoordsOffset()+3] = 0.0;
    return true;
}

bool SphericalJoint::normalizeJointPosCoords(iDynTree::Span<double> jntPos) const
{
    if (jntPos.size() < this->getPosCoordsOffset()+4) return false;
    Vector4 q;
    q(0) = jntPos[this->getPosCoordsOffset()+0];
    q(1) = jntPos[this->getPosCoordsOffset()+1];
    q(2) = jntPos[this->getPosCoordsOffset()+2];
    q(3) = jntPos[this->getPosCoordsOffset()+3];
    normalizeQuaternion(q);
    jntPos[this->getPosCoordsOffset()+0] = q(0);
    jntPos[this->getPosCoordsOffset()+1] = q(1);
    jntPos[this->getPosCoordsOffset()+2] = q(2);
    jntPos[this->getPosCoordsOffset()+3] = q(3);
    return true;
}

}
