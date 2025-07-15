// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/RevoluteSO2Joint.h>

#include <iDynTree/Axis.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/Twist.h>
#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <cmath>
#include <cfloat>
#include <cfloat>

namespace iDynTree
{

RevoluteSO2Joint::RevoluteSO2Joint():
        link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX), link1_X_link2_at_rest(Transform::Identity()),
        rotation_axis_wrt_link1(Axis(Direction(1.0, 0.0, 0.0), Position::Zero()))
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(1.0, 0.0); // Initialize with angle = 0 (complex number = 1 + 0i)
    this->disablePosLimits();
    this->resetJointDynamics();
}

RevoluteSO2Joint::RevoluteSO2Joint(const LinkIndex _link1, const LinkIndex _link2,
                             const Transform& _link1_X_link2, const Axis& _rotation_axis_wrt_link1):
                             link1(_link1), link2(_link2), link1_X_link2_at_rest(_link1_X_link2),
                             rotation_axis_wrt_link1(_rotation_axis_wrt_link1)
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(1.0, 0.0); // Initialize with angle = 0 (complex number = 1 + 0i)
    this->disablePosLimits();
    this->resetJointDynamics();
}

RevoluteSO2Joint::RevoluteSO2Joint(const Transform& _link1_X_link2, const Axis& _rotation_axis_wrt_link1):
                             link1(LINK_INVALID_INDEX), link2(LINK_INVALID_INDEX), link1_X_link2_at_rest(_link1_X_link2),
                             rotation_axis_wrt_link1(_rotation_axis_wrt_link1)
{
    this->setPosCoordsOffset(0);
    this->setDOFsOffset(0);

    this->resetAxisBuffers();
    this->resetBuffers(1.0, 0.0); // Initialize with angle = 0 (complex number = 1 + 0i)
    this->disablePosLimits();
    this->resetJointDynamics();
}

RevoluteSO2Joint::RevoluteSO2Joint(const RevoluteSO2Joint& other):
                             link1(other.link1), link2(other.link2),
                             link1_X_link2_at_rest(other.link1_X_link2_at_rest),
                             rotation_axis_wrt_link1(other.rotation_axis_wrt_link1),
                             m_hasPosLimits(other.m_hasPosLimits),
                             m_minPos(other.m_minPos), m_maxPos(other.m_maxPos),
                             m_joint_dynamics_type(other.m_joint_dynamics_type),
                             m_damping(other.m_damping), m_static_friction(other.m_static_friction)
{
    this->setPosCoordsOffset(other.getPosCoordsOffset());
    this->setDOFsOffset(other.getDOFsOffset());

    this->resetAxisBuffers();
    this->resetBuffers(1.0, 0.0); // Initialize with angle = 0 (complex number = 1 + 0i)
}

RevoluteSO2Joint::~RevoluteSO2Joint()
{
}

IJoint* RevoluteSO2Joint::clone() const
{
    return (IJoint *) new RevoluteSO2Joint(*this);
}

double RevoluteSO2Joint::complexToAngle(const double q_real, const double q_imag) const
{
    return std::atan2(q_imag, q_real);
}

void RevoluteSO2Joint::normalizeComplex(double& q_real, double& q_imag) const
{
    double norm = std::sqrt(q_real * q_real + q_imag * q_imag);
    if (norm > 1e-12) // Avoid division by zero
    {
        q_real /= norm;
        q_imag /= norm;
    }
    else
    {
        // If norm is too small, default to angle = 0
        q_real = 1.0;
        q_imag = 0.0;
    }
}

void RevoluteSO2Joint::updateBuffers(const double new_q_real, const double new_q_imag) const
{
    if( new_q_real != q_real_previous || new_q_imag != q_imag_previous )
    {
        resetBuffers(new_q_real, new_q_imag);
    }

    return;
}

void RevoluteSO2Joint::resetBuffers(const double new_q_real, const double new_q_imag) const
{
    // Normalize the complex number to ensure it's on the unit circle
    double norm_q_real = new_q_real;
    double norm_q_imag = new_q_imag;
    normalizeComplex(norm_q_real, norm_q_imag);
    
    // Convert to angle
    double angle = complexToAngle(norm_q_real, norm_q_imag);
    
    // Compute transforms using the angle
    this->link1_X_link2 = rotation_axis_wrt_link1.getRotationTransform(angle) * link1_X_link2_at_rest;
    this->link2_X_link1 = link1_X_link2.inverse();

    q_real_previous = new_q_real;
    q_imag_previous = new_q_imag;
}

void RevoluteSO2Joint::resetAxisBuffers() const
{
    this->S_link1_link2 = -(rotation_axis_wrt_link1).getRotationTwist(1.0);
    this->S_link2_link1 = (link1_X_link2_at_rest.inverse()*rotation_axis_wrt_link1).getRotationTwist(1.0);
}

void RevoluteSO2Joint::setRestTransform(const Transform& _link1_X_link2)
{
    this->link1_X_link2_at_rest = _link1_X_link2;

    this->resetAxisBuffers();
}

LinkIndex RevoluteSO2Joint::getFirstAttachedLink() const
{
    return link1;
}

LinkIndex RevoluteSO2Joint::getSecondAttachedLink() const
{
    return link2;
}

Transform RevoluteSO2Joint::getRestTransform(const LinkIndex p_linkA, const LinkIndex p_linkB) const
{
    if( p_linkA == this->link1 )
    {
        assert(p_linkB == link2);
        return this->link1_X_link2_at_rest;
    }
    else
    {
        assert(p_linkA == link2);
        assert(p_linkB == link1);
        return this->link1_X_link2_at_rest.inverse();
    }
}

const Transform & RevoluteSO2Joint::getTransform(const VectorDynSize & jntPos,
                                                    const LinkIndex p_linkA,
                                                    const LinkIndex p_linkB) const
{
    const double q_real = jntPos(this->getPosCoordsOffset());
    const double q_imag = jntPos(this->getPosCoordsOffset() + 1);

    updateBuffers(q_real, q_imag);

    if( p_linkA == this->link1 )
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

TransformDerivative RevoluteSO2Joint::getTransformDerivative(const VectorDynSize& jntPos,
                                                                const LinkIndex linkA,
                                                                const LinkIndex linkB,
                                                                const int posCoord_i) const
{
    const double q_real = jntPos(this->getPosCoordsOffset());
    const double q_imag = jntPos(this->getPosCoordsOffset() + 1);
    
    // Normalize the complex number
    double norm_q_real = q_real;
    double norm_q_imag = q_imag;
    normalizeComplex(norm_q_real, norm_q_imag);
    
    // Compute angle and its derivatives w.r.t. the position coordinates
    double angle = complexToAngle(norm_q_real, norm_q_imag);
    
    double dangle_dcoord;
    double norm = std::sqrt(q_real*q_real + q_imag*q_imag);
    if (norm > 1e-12)
    {
        // d(atan2(y,x))/dx = -y/(x^2 + y^2), d(atan2(y,x))/dy = x/(x^2 + y^2)
        double norm_sq = norm * norm;
        if (posCoord_i == 0) // Derivative w.r.t. real part
        {
            dangle_dcoord = -q_imag / norm_sq;
        }
        else if (posCoord_i == 1) // Derivative w.r.t. imaginary part
        {
            dangle_dcoord = q_real / norm_sq;
        }
        else
        {
            dangle_dcoord = 0.0; // Invalid coordinate
        }
    }
    else
    {
        dangle_dcoord = 0.0;
    }

    // Compute derivative using same pattern as RevoluteJoint but scaled by chain rule
    TransformDerivative transDerivativeAngle = rotation_axis_wrt_link1.getRotationTransformDerivative(angle);
    Matrix4x4 link1_dX_link2_hom;
    TransformDerivative link1_dX_link2;
    toEigen(link1_dX_link2_hom) = toEigen((transDerivativeAngle * link1_X_link2_at_rest).asHomogeneousTransformDerivative()) * dangle_dcoord;
    link1_dX_link2.fromHomogeneousTransformDerivative(link1_dX_link2_hom);

    if( linkA == this->link1 )
    {
        return link1_dX_link2;
    }
    else
    {
        updateBuffers(q_real, q_imag);
        TransformDerivative linkA_dX_linkB = link1_dX_link2.derivativeOfInverse(this->link1_X_link2);
        return linkA_dX_linkB;
    }
}

SpatialMotionVector RevoluteSO2Joint::getMotionSubspaceVector(int dof_i,
                                                                 const LinkIndex p_linkA,
                                                                 const LinkIndex p_linkB) const
{
    // This joint has only 1 DOF (angular velocity), so dof_i should be 0
    assert(dof_i == 0);
    
    if( p_linkA == link2 )
    {
        return this->S_link2_link1;
    }
    else
    {
        return this->S_link1_link2;
    }
}

Axis RevoluteSO2Joint::getAxis(const LinkIndex child,
                                  const LinkIndex /*parent*/) const
{
    if( child == link1 )
    {
        return rotation_axis_wrt_link1.reverse();
    }
    else
    {
        assert(child == link2);
        return link1_X_link2_at_rest.inverse()*rotation_axis_wrt_link1;
    }
}

void RevoluteSO2Joint::setAttachedLinks(const LinkIndex _link1, const LinkIndex _link2)
{
    this->link1 = _link1;
    this->link2 = _link2;
}

void RevoluteSO2Joint::setAxis(const Axis& revoluteAxis_wrt_link1)
{
    this->rotation_axis_wrt_link1 = revoluteAxis_wrt_link1;

    this->resetAxisBuffers();
}

void RevoluteSO2Joint::setAxis(const Axis &revoluteAxis,
                                  const LinkIndex child, const LinkIndex /*parent*/)
{
    if( child == link1 )
    {
        rotation_axis_wrt_link1 = revoluteAxis.reverse();
    }
    else
    {
        assert(child == link2);
        rotation_axis_wrt_link1 = link1_X_link2_at_rest*revoluteAxis;
    }

    this->resetAxisBuffers();
}

void RevoluteSO2Joint::computeChildVelAcc(const VectorDynSize & jntPos,
                                             const VectorDynSize & jntVel,
                                             const VectorDynSize & jntAcc,
                                             LinkVelArray & linkVels,
                                             LinkAccArray & linkAccs,
                                             const LinkIndex child, const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());
    double d2ang = jntAcc(this->getDOFsOffset());

    if( child == link1 )
    {
        linkVels(child) = getMotionSubspaceVector(0,child,parent)*dang + 
                         getTransform(jntPos,child,parent)*linkVels(parent);
        linkAccs(child) = getMotionSubspaceVector(0,child,parent)*d2ang + 
                         getTransform(jntPos,child,parent)*linkAccs(parent);
    }
    else
    {
        assert(child == link2);
        linkVels(child) = getMotionSubspaceVector(0,child,parent)*dang + 
                         getTransform(jntPos,child,parent)*linkVels(parent);
        linkAccs(child) = getMotionSubspaceVector(0,child,parent)*d2ang + 
                         getTransform(jntPos,child,parent)*linkAccs(parent);
    }
}

void RevoluteSO2Joint::computeChildVel(const VectorDynSize & jntPos,
                                          const VectorDynSize & jntVel,
                                          LinkVelArray & linkVels,
                                          const LinkIndex child, const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());

    linkVels(child) = getMotionSubspaceVector(0,child,parent)*dang + 
                     getTransform(jntPos,child,parent)*linkVels(parent);
}

void RevoluteSO2Joint::computeChildAcc(const VectorDynSize & jntPos,
                                          const VectorDynSize & jntVel,
                                          const LinkVelArray & linkVels,
                                          const VectorDynSize & jntAcc,
                                          LinkAccArray & linkAccs,
                                          const LinkIndex child,
                                          const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());
    double d2ang = jntAcc(this->getDOFsOffset());

    SpatialMotionVector S = getMotionSubspaceVector(0,child,parent);
    Transform X = getTransform(jntPos,child,parent);
    
    // Compute bias acceleration term: v_child * S*dq
    SpatialMotionVector vj = S * dang;
    
    linkAccs(child) = S*d2ang + X*linkAccs(parent) + linkVels(child)*vj;
}

void RevoluteSO2Joint::computeChildBiasAcc(const VectorDynSize & jntPos,
                                              const VectorDynSize & jntVel,
                                              const LinkVelArray & linkVels,
                                              LinkAccArray & linkBiasAccs,
                                              const LinkIndex child,
                                              const LinkIndex parent) const
{
    double dang = jntVel(this->getDOFsOffset());

    SpatialMotionVector S = getMotionSubspaceVector(0,child,parent);
    Transform X = getTransform(jntPos,child,parent);
    
    // Compute bias acceleration term: v_child * S*dq
    SpatialMotionVector vj = S * dang;
    
    linkBiasAccs(child) = X*linkBiasAccs(parent) + linkVels(child)*vj;
}

void RevoluteSO2Joint::computeJointTorque(const VectorDynSize & jntPos, const Wrench & internalWrench,
                                             const LinkIndex linkThatAppliesWrench, const LinkIndex linkOnWhichWrenchIsApplied,
                                             VectorDynSize & jntTorques) const
{
    SpatialMotionVector S = getMotionSubspaceVector(0, linkOnWhichWrenchIsApplied, linkThatAppliesWrench);
    jntTorques(this->getDOFsOffset()) = S.dot(internalWrench);
}

// LIMITS METHODS
void RevoluteSO2Joint::disablePosLimits()
{
    m_hasPosLimits = false;
    m_minPos = -DBL_MAX;
    m_maxPos = +DBL_MAX;
}

void RevoluteSO2Joint::resetJointDynamics()
{
    m_joint_dynamics_type = NoJointDynamics;
    m_damping = 0.0;
    m_static_friction = 0.0;
}

bool RevoluteSO2Joint::hasPosLimits() const
{
    return m_hasPosLimits;
}

bool RevoluteSO2Joint::enablePosLimits(const bool enable)
{
    m_hasPosLimits = enable;
    return true;
}

bool RevoluteSO2Joint::getPosLimits(const size_t _index, double & min, double & max) const
{
    if( _index >= 2 ) return false;
    
    min = m_minPos;
    max = m_maxPos;
    return true;
}

double RevoluteSO2Joint::getMinPosLimit(const size_t _index) const
{
    if( _index >= 2 ) return 0.0;
    
    return m_minPos;
}

double RevoluteSO2Joint::getMaxPosLimit(const size_t _index) const
{
    if( _index >= 2 ) return 0.0;
    
    return m_maxPos;
}

bool RevoluteSO2Joint::setPosLimits(const size_t _index, double min, double max)
{
    if( _index >= 2 ) return false;
    
    m_minPos = min;
    m_maxPos = max;
    return true;
}

// DYNAMICS METHODS
JointDynamicsType RevoluteSO2Joint::getJointDynamicsType() const
{
    return m_joint_dynamics_type;
}

bool RevoluteSO2Joint::setJointDynamicsType(const JointDynamicsType dynamicsType)
{
    m_joint_dynamics_type = dynamicsType;
    return true;
}

double RevoluteSO2Joint::getDamping(const size_t _index) const
{
    if( _index >= 1 ) return 0.0;
    
    return m_damping;
}

double RevoluteSO2Joint::getStaticFriction(const size_t _index) const
{
    if( _index >= 1 ) return 0.0;
    
    return m_static_friction;
}

bool RevoluteSO2Joint::setDamping(const size_t _index, double damping)
{
    if( _index >= 1 ) return false;
    
    m_damping = damping;
    return true;
}

bool RevoluteSO2Joint::setStaticFriction(const size_t _index, double staticFriction)
{
    if( _index >= 1 ) return false;
    
    m_static_friction = staticFriction;
    return true;
}

// Utility methods
double RevoluteSO2Joint::getCurrentAngle(const VectorDynSize & jntPos) const
{
    const double q_real = jntPos(this->getPosCoordsOffset());
    const double q_imag = jntPos(this->getPosCoordsOffset() + 1);
    
    double norm_q_real = q_real;
    double norm_q_imag = q_imag;
    normalizeComplex(norm_q_real, norm_q_imag);
    
    return complexToAngle(norm_q_real, norm_q_imag);
}

void RevoluteSO2Joint::computeChildPosVelAcc(const VectorDynSize & jntPos,
                                              const VectorDynSize & jntVel,
                                              const VectorDynSize & jntAcc,
                                              LinkPositions & linkPositions,
                                              LinkVelArray & linkVels,
                                              LinkAccArray & linkAccs,
                                              const LinkIndex child,
                                              const LinkIndex parent) const
{
    // For SO2 joint, we have 1 DOF (angular velocity) but 2 position coordinates
    double dang = jntVel(this->getDOFsOffset());
    double d2ang = jntAcc(this->getDOFsOffset());

    const Transform & child_X_parent = this->getTransform(jntPos,child,parent);
    const Transform & parent_X_child = this->getTransform(jntPos,parent,child);

    // Propagate position : position of the frame is expressed as
    // transform between the link frame and a reference frame :
    // ref_H_child  = ref_H_parent*parent_H_child
    linkPositions(child) = linkPositions(parent)*parent_X_child;

    // Propagate twist and spatial acceleration: for a revolute SO2 joint
    // we implement equation 5.14 and 5.15 of Feathestone RBDA, 2008
    iDynTree::SpatialMotionVector S = this->getMotionSubspaceVector(0,child);

    SpatialMotionVector vj = S*dang;
    linkVels(child) = child_X_parent*linkVels(parent) + vj;
    linkAccs(child) = child_X_parent*linkAccs(parent) + S*d2ang + linkVels(child)*vj;
}

void RevoluteSO2Joint::setJointPositionFromAngle(VectorDynSize & jntPos, double angle) const
{
    jntPos(this->getPosCoordsOffset()) = std::cos(angle);
    jntPos(this->getPosCoordsOffset() + 1) = std::sin(angle);
}

bool RevoluteSO2Joint::getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos, 
                                                          MatrixView<double>& positionDerivative_J_velocity) const
{
    // RevoluteSO2 joint has 2 position coordinates (real, imag) and 1 DOF (angular velocity)
    // The Jacobian maps angular velocity to derivative of complex number:
    // d/dt [cos(θ)] = [-sin(θ)] * dθ/dt
    // d/dt [sin(θ)] = [ cos(θ)] * dθ/dt
    
    if (positionDerivative_J_velocity.rows() != 2 || positionDerivative_J_velocity.cols() != 1) {
        return false; // Wrong size
    }
    
    if (jntPos.size() < this->getPosCoordsOffset() + 2) {
        return false; // Not enough joint position data
    }
    
    // Extract the current complex coordinates
    const double q_real = jntPos[this->getPosCoordsOffset()];
    const double q_imag = jntPos[this->getPosCoordsOffset() + 1];
    
    // Normalize to ensure we're on the unit circle
    double norm_q_real = q_real;
    double norm_q_imag = q_imag;
    normalizeComplex(norm_q_real, norm_q_imag);
    
    // The Jacobian is:
    // J = [-sin(θ)]  = [-q_imag_normalized]
    //     [ cos(θ)]    [ q_real_normalized]
    positionDerivative_J_velocity(0, 0) = -norm_q_imag; // d/dt(real) = -sin(θ) * dθ/dt
    positionDerivative_J_velocity(1, 0) =  norm_q_real; // d/dt(imag) =  cos(θ) * dθ/dt
    
    return true;
}

bool RevoluteSO2Joint::setJointPosCoordsToRest(iDynTree::Span<double> jntPos) const
{
    // RevoluteSO2 joint has 2 position coordinates: (real, imag) representing cos(θ), sin(θ)
    // Rest position corresponds to θ = 0, so (real, imag) = (1, 0)
    if (jntPos.size() < this->getPosCoordsOffset() + 2) {
        return false; // Not enough data in the span
    }
    jntPos[this->getPosCoordsOffset()] = 1.0; // cos(0) = 1
    jntPos[this->getPosCoordsOffset() + 1] = 0.0; // sin(0) = 0
    return true;
}

bool RevoluteSO2Joint::normalizeJointPosCoords(iDynTree::Span<double> jntPos) const
{
    // RevoluteSO2 joint uses a constrained representation (unit complex number)
    // Normalize to ensure it lies on the unit circle
    if (jntPos.size() < this->getPosCoordsOffset() + 2) {
        return false; // Not enough data in the span
    }
    
    double q_real = jntPos[this->getPosCoordsOffset()];
    double q_imag = jntPos[this->getPosCoordsOffset() + 1];
    
    // Use the existing helper function to normalize
    normalizeComplex(q_real, q_imag);
    
    // Update the position coordinates with normalized values
    jntPos[this->getPosCoordsOffset()] = q_real;
    jntPos[this->getPosCoordsOffset() + 1] = q_imag;
    
    return true;
}

}
