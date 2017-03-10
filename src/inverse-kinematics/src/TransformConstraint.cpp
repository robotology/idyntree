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

    TransformConstraint::TransformConstraint(const std::string& constrainedFrameName, TransformConstraintType type): m_type(type), m_frameName(constrainedFrameName) {}

    TransformConstraint TransformConstraint::positionConstraint(const std::string& frameName, const iDynTree::Position &position)
    {
        TransformConstraint transformConstraint(frameName, TransformConstraintTypePosition);
        transformConstraint.m_transform.setPosition(position);
        return transformConstraint;
    }

    TransformConstraint TransformConstraint::rotationConstraint(const std::string& frameName, const iDynTree::Rotation &rotation)
    {
        TransformConstraint transformConstraint(frameName, TransformConstraintTypeRotation);
        transformConstraint.m_transform.setRotation(rotation);
        return transformConstraint;
    }

    TransformConstraint TransformConstraint::fullTransformConstraint(const std::string& frameName, const iDynTree::Position &position, const iDynTree::Rotation &rotation)
    {
        iDynTree::Transform transform(rotation, position);
        return fullTransformConstraint(frameName, transform);
    }

    TransformConstraint TransformConstraint::fullTransformConstraint(const std::string& frameName, const iDynTree::Transform &_transform)
    {
        TransformConstraint transform(frameName, TransformConstraintTypeFullTransform);
        transform.m_transform = _transform;
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
    const iDynTree::Rotation& TransformConstraint::getRotation() const { return m_transform.getRotation(); }
    const iDynTree::Transform& TransformConstraint::getTransform() const { return m_transform; }
    const std::string& TransformConstraint::getFrameName() const { return m_frameName; }
}
}
