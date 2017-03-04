/*!
 * @file Transform.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Transform.h"
#include <iDynTree/Core/Transform.h>

namespace kinematics {

    Transform::Transform(std::string frameName, TransformType type): m_type(type), m_frameName(frameName) {}

    Transform Transform::positionConstraint(std::string frameName, const iDynTree::Position &position)
    {
        Transform transform(frameName, ConstraintTypePosition);
        transform.m_transform.setPosition(position);
        return transform;
    }

    Transform Transform::rotationConstraint(std::string frameName, const iDynTree::Rotation &rotation)
    {
        Transform transform(frameName, ConstraintTypeRotation);
        transform.m_transform.setRotation(rotation);
        return transform;
    }

    Transform Transform::transformConstraint(std::string frameName, const iDynTree::Position &position, const iDynTree::Rotation &rotation)
    {
        iDynTree::Transform transform(rotation, position);
        return transformConstraint(frameName, transform);
    }

    Transform Transform::transformConstraint(std::string frameName, const iDynTree::Transform &_transform)
    {
        Transform transform(frameName, ConstraintTypeTransform);
        transform.m_transform = _transform;
        return transform;
    }

    unsigned Transform::getSize() const
    {
        switch (m_type) {
            case ConstraintTypePosition:
                return 3;
            case ConstraintTypeRotation:
                return 4;
            case ConstraintTypeTransform:
                return 7;
        }
    }

    Transform::TransformType Transform::getType() const { return m_type; }
    bool Transform::hasPositionConstraint() const { return m_type == ConstraintTypePosition || m_type == ConstraintTypeTransform; }
    bool Transform::hasRotationConstraint() const { return m_type == ConstraintTypeRotation || m_type == ConstraintTypeTransform; }

    const iDynTree::Position& Transform::getPosition() const { return m_transform.getPosition(); }
    const iDynTree::Rotation& Transform::getRotation() const { return m_transform.getRotation(); }
    const iDynTree::Transform& Transform::getTransform() const { return m_transform; }
    const std::string& Transform::getFrameName() const { return m_frameName; }
}

