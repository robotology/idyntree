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

#include <cassert>
#include <cmath>

namespace iDynTree
{

SphericalJoint::SphericalJoint():
        link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX),
        link1_X_link2_at_rest(Transform::Identity())
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
                             link1_X_link2_at_rest(_link1_X_link2)
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
                             link1_X_link2_at_rest(other.link1_X_link2_at_rest)
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
    this->resetAxisBuffers();
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
        q(0) /= norm;
        q(1) /= norm;
        q(2) /= norm;
        q(3) /= norm;
    }
    else
    {
        // Fallback to identity quaternion if norm is too small
        q(0) = 1.0; q(1) = 0.0; q(2) = 0.0; q(3) = 0.0;
    }
}

Transform SphericalJoint::quaternionToTransform(const Vector4& q) const
{
    Transform result;

    // Set translation to zero (spherical joint doesn't allow translation)
    result.setPosition(Position::Zero());

    // Convert quaternion (w,x,y,z) to rotation matrix using iDynTree's convention
    Rotation rot;
    rot.fromQuaternion(q);
    result.setRotation(rot);

    return result;
}

void SphericalJoint::resetBuffers(const Vector4& new_q) const
{
    this->q_previous = new_q;

    // Create transform from quaternion
    Transform quat_transform = quaternionToTransform(new_q);

    this->link1_X_link2 = this->link1_X_link2_at_rest * quat_transform;
    this->link2_X_link1 = this->link1_X_link2.inverse();
}

void SphericalJoint::updateBuffers(const Vector4& new_q) const
{
    // Check if quaternion has changed
    bool quaternion_changed = false;
    for (int i = 0; i < 4; i++)
    {
        if (std::abs(new_q(i) - this->q_previous(i)) > 1e-12)
        {
            quaternion_changed = true;
            break;
        }
    }

    if (quaternion_changed)
    {
        this->resetBuffers(new_q);
    }
}

void SphericalJoint::resetAxisBuffers() const
{
    // Motion subspace for a spherical joint is pure angular (no linear part)
    // and, by the IJoint contract, it must be expressed in the CHILD frame.
    // Since we return S based on which link is the child in getMotionSubspaceVector,
    // both cached arrays must contain the same three unit angular basis vectors
    // (x, y, z) with zero linear components. No sign inversion is required when
    // swapping child/parent; the expression frame changes, not the sign.

    LinearMotionVector3 v_lin; v_lin.zero();
    AngularMotionVector3 wx, wy, wz;
    wx(0) = 1.0; wx(1) = 0.0; wx(2) = 0.0;
    wy(0) = 0.0; wy(1) = 1.0; wy(2) = 0.0;
    wz(0) = 0.0; wz(1) = 0.0; wz(2) = 1.0;

    // Child = link2, Parent = link1
    this->S_link1_link2[0] = SpatialMotionVector(v_lin, wx);
    this->S_link1_link2[1] = SpatialMotionVector(v_lin, wy);
    this->S_link1_link2[2] = SpatialMotionVector(v_lin, wz);

    // Child = link1, Parent = link2 (same basis, expressed in that child frame)
    this->S_link2_link1[0] = SpatialMotionVector(v_lin, wx);
    this->S_link2_link1[1] = SpatialMotionVector(v_lin, wy);
    this->S_link2_link1[2] = SpatialMotionVector(v_lin, wz);
}

Transform SphericalJoint::getRestTransform(const LinkIndex child, const LinkIndex parent) const
{
    if( child == this->link1 )
    {
        assert( parent == this->link2 );
        return this->link1_X_link2_at_rest.inverse();
    }
    else
    {
        assert( child == this->link2 );
        assert( parent == this->link1 );
        return this->link1_X_link2_at_rest;
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

    // Normalize quaternion
    normalizeQuaternion(q);

    // Update internal buffers
    this->updateBuffers(q);

    if( child == this->link1 )
    {
        assert( parent == this->link2 );
        return this->link2_X_link1;
    }
    else
    {
        assert( child == this->link2 );
        assert( parent == this->link1 );
        return this->link1_X_link2;
    }
}

TransformDerivative SphericalJoint::getTransformDerivative(const VectorDynSize & jntPos,
                                                          const LinkIndex child,
                                                          const LinkIndex parent,
                                                          const int posCoord_i) const
{
    // For spherical joint, this is complex due to quaternion parameterization
    // For now, return zero - this would need proper implementation for advanced features
    return TransformDerivative::Zero();
}

SpatialMotionVector SphericalJoint::getMotionSubspaceVector(int dof_i,
                                                           const LinkIndex child,
                                                           const LinkIndex parent) const
{
    // Convention: the joint velocity coordinates ν (size 3) represent
    // the angular velocity of link2 w.r.t. link1 expressed in link2 frame.
    // Therefore:
    // - For child=link2,parent=link1, {}^C s_{P,C} is constant with columns [0; e_x], [0; e_y], [0; e_z].
    // - For child=link1,parent=link2, we must express v_{1,2} in frame 1 as
    //   [0; - R^1_2 ω^2], where R^1_2 is the current rotation from frame 2 to 1.
    if( child == this->link1 )
    {
        assert( parent == this->link2 );

        // Use the cached current transform (updated by the last getTransform call)
        const Rotation & R12 = this->link1_X_link2.getRotation();

        LinearMotionVector3 v_lin; v_lin.zero();
        AngularMotionVector3 v_ang;

        // R * e_i is the i-th column of R
        v_ang(0) = - R12(0, dof_i);
        v_ang(1) = - R12(1, dof_i);
        v_ang(2) = - R12(2, dof_i);

        return SpatialMotionVector(v_lin, v_ang);
    }
    else
    {
        assert( child == this->link2 );
        assert( parent == this->link1 );
        return this->S_link1_link2[dof_i];
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

    // Add bias acceleration due to joint velocity (Coriolis terms)
    SpatialAcc bias_acc = SpatialAcc::Zero();
    // For spherical joint, bias acceleration comes from cross products of angular velocities
    // This is a simplified implementation - full implementation would need proper Coriolis terms

    linkAccs(child) = child_X_parent * linkAccs(parent) + joint_acc + bias_acc;
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

    // Add bias acceleration
    SpatialAcc bias_acc = SpatialAcc::Zero();

    linkAccs(child) = child_X_parent * linkAccs(parent) + joint_acc + bias_acc;
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

    // For spherical joint, bias acceleration includes Coriolis terms
    // This is a simplified implementation
    SpatialAcc bias_acc = SpatialAcc::Zero();

    linkBiasAccs(child) = child_X_parent * linkBiasAccs(parent) + bias_acc;
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

bool SphericalJoint::enablePosLimits(const bool enable)
{
    // Position limits are not supported for spherical joints
    return false;
}

bool SphericalJoint::getPosLimits(const size_t _index, double & min, double & max) const
{
    return false;
}

double SphericalJoint::getMinPosLimit(const size_t _index) const
{
    return 0.0;
}

double SphericalJoint::getMaxPosLimit(const size_t _index) const
{
    return 0.0;
}

bool SphericalJoint::setPosLimits(const size_t _index, double min, double max)
{
    return false;
}

JointDynamicsType SphericalJoint::getJointDynamicsType() const
{
    return NoJointDynamics;
}

bool SphericalJoint::setJointDynamicsType(const JointDynamicsType enable)
{
    // Joint dynamics are not supported for spherical joints
    return false;
}

bool SphericalJoint::setDamping(const size_t _index, double damping)
{
    return false;
}

bool SphericalJoint::setStaticFriction(const size_t _index, double staticFriction)
{
    return false;
}

double SphericalJoint::getDamping(const size_t _index) const
{
    return 0.0;
}

double SphericalJoint::getStaticFriction(const size_t _index) const
{
    return 0.0;
}

bool SphericalJoint::getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos,
                                                          iDynTree::MatrixView<double>& jac) const
{
    // For a spherical joint, the position derivative velocity Jacobian relates
    // the time derivative of quaternion coordinates to angular velocity
    // This is typically used for dynamics computations
    if (jac.rows() != 4 || jac.cols() != 3)
    {
        return false;
    }

    // Extract quaternion from joint positions
    if (jntPos.size() < this->getPosCoordsOffset() + 4)
    {
        return false;
    }

    size_t offset = this->getPosCoordsOffset();
    Vector4 q;
    q(0) = jntPos[offset + 0]; // qw (real part)
    q(1) = jntPos[offset + 1]; // qx
    q(2) = jntPos[offset + 2]; // qy
    q(3) = jntPos[offset + 3]; // qz

    // The relationship between quaternion derivative and angular velocity is:
    // dq/dt = 0.5 * Q(q) * omega
    // where Q(q) is the quaternion matrix and omega is the angular velocity

    // Q(q) matrix for right quaternion multiplication
    jac(0, 0) = -q(1); jac(0, 1) = -q(2); jac(0, 2) = -q(3);
    jac(1, 0) =  q(0); jac(1, 1) = -q(3); jac(1, 2) =  q(2);
    jac(2, 0) =  q(3); jac(2, 1) =  q(0); jac(2, 2) = -q(1);
    jac(3, 0) = -q(2); jac(3, 1) =  q(1); jac(3, 2) =  q(0);

    // Scale by 0.5
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            jac(i, j) *= 0.5;
        }
    }

    return true;
}

bool SphericalJoint::setJointPosCoordsToRest(iDynTree::Span<double> jntPos) const
{
    // Set the quaternion to identity (no rotation)
    if (jntPos.size() < this->getPosCoordsOffset() + 4)
    {
        return false;
    }

    size_t offset = this->getPosCoordsOffset();
    jntPos[offset + 0] = 1.0; // qw (real part)
    jntPos[offset + 1] = 0.0; // qx
    jntPos[offset + 2] = 0.0; // qy
    jntPos[offset + 3] = 0.0; // qz

    return true;
}

bool SphericalJoint::normalizeJointPosCoords(iDynTree::Span<double> jntPos) const
{
    // Normalize the quaternion to unit length
    if (jntPos.size() < this->getPosCoordsOffset() + 4)
    {
        return false;
    }

    size_t offset = this->getPosCoordsOffset();
    double norm = sqrt(jntPos[offset + 0] * jntPos[offset + 0] +
                      jntPos[offset + 1] * jntPos[offset + 1] +
                      jntPos[offset + 2] * jntPos[offset + 2] +
                      jntPos[offset + 3] * jntPos[offset + 3]);

    if (norm < 1e-12)
    {
        // If norm is too small, set to identity quaternion
        jntPos[offset + 0] = 1.0;  // qw (real part)
        jntPos[offset + 1] = 0.0;  // qx
        jntPos[offset + 2] = 0.0;  // qy
        jntPos[offset + 3] = 0.0;  // qz
    }
    else
    {
        // Normalize
        jntPos[offset + 0] /= norm;
        jntPos[offset + 1] /= norm;
        jntPos[offset + 2] /= norm;
        jntPos[offset + 3] /= norm;
    }

    return true;
}

}
