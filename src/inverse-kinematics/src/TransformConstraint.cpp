/*!
 * @file Transform.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#include "TransformConstraint.h"
#include <iDynTree/Core/Transform.h>

namespace internal {
namespace kinematics {

    TransformConstraint::TransformConstraint(const std::string& constrainedFrameName,
                                             TransformConstraintType type)
    : m_type(type)
    , m_frameName(constrainedFrameName)
    , m_posWeight(1.0)
    , m_rotWeight(1.0)
    , m_resolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone)
    {}

    TransformConstraint TransformConstraint::positionConstraint(const std::string& frameName, const iDynTree::Position &position, const double posWeight)
    {
        TransformConstraint transformConstraint(frameName, TransformConstraintTypePosition);
        transformConstraint.m_transform.setPosition(position);
        transformConstraint.m_posWeight = posWeight;
        return transformConstraint;
    }

    TransformConstraint TransformConstraint::rotationConstraint(const std::string& frameName, const iDynTree::Rotation &rotation, const double rotWeight)
    {
        TransformConstraint transformConstraint(frameName, TransformConstraintTypeRotation);
        transformConstraint.m_transform.setRotation(rotation);
        transformConstraint.m_rotWeight = rotWeight;
        return transformConstraint;
    }

    TransformConstraint TransformConstraint::fullTransformConstraint(const std::string& frameName,
                                                                     const iDynTree::Position &position,
                                                                     const iDynTree::Rotation &rotation,
                                                                     const double posWeight,
                                                                     const double rotWeight)
    {
        iDynTree::Transform transform(rotation, position);
        return fullTransformConstraint(frameName, transform,posWeight,rotWeight);
    }

    TransformConstraint TransformConstraint::fullTransformConstraint(const std::string& frameName,
                                                                     const iDynTree::Transform &_transform,
                                                                     const double posWeight,
                                                                     const double rotWeight)
    {
        TransformConstraint transform(frameName, TransformConstraintTypeFullTransform);
        transform.m_transform = _transform;
        transform.m_posWeight = posWeight;
        transform.m_rotWeight = rotWeight;
        return transform;
    }

    unsigned TransformConstraint::getSize() const
    {
        switch (m_type) {
            case TransformConstraintTypePosition:
                return 3;
            case TransformConstraintTypeRotation:
                return 4;
            case TransformConstraintTypeFullTransform:
                return 7;
        }
    }

    TransformConstraint::TransformConstraintType TransformConstraint::getType() const { return m_type; }
    bool TransformConstraint::hasPositionConstraint() const { return m_type == TransformConstraintTypePosition || m_type == TransformConstraintTypeFullTransform; }
    bool TransformConstraint::hasRotationConstraint() const { return m_type == TransformConstraintTypeRotation || m_type == TransformConstraintTypeFullTransform; }

    const iDynTree::Position& TransformConstraint::getPosition() const { return m_transform.getPosition(); }
    void TransformConstraint::setPosition(iDynTree::Position& newPos) { m_transform.setPosition(newPos); }
    const iDynTree::Rotation& TransformConstraint::getRotation() const { return m_transform.getRotation(); }
    void TransformConstraint::setRotation(iDynTree::Rotation& newRot) { m_transform.setRotation(newRot); }
    const iDynTree::Transform& TransformConstraint::getTransform() const { return m_transform; }
    const std::string& TransformConstraint::getFrameName() const { return m_frameName; }
    const double TransformConstraint::getPositionWeight() const { return m_posWeight; }
    void TransformConstraint::setPositionWeight(const double newPosWeight) { if (newPosWeight >= 0.0) m_posWeight = newPosWeight; }
    const double TransformConstraint::getRotationWeight() const { return m_rotWeight; }
    void TransformConstraint::setRotationWeight(const double newRotWeight) { if (newRotWeight >= 0.0) m_posWeight = newRotWeight; }
    
    void TransformConstraint::setTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraint mode){ m_resolutionMode = mode; }
    iDynTree::InverseKinematicsTreatTargetAsConstraint TransformConstraint::targetResolutionMode() const{ return m_resolutionMode; }
    
}
}
