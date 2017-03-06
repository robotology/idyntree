/*!
 * @file Transform.cpp
 * @author Francesco Romano
 * @copyright 2016 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2016
 *
 */

#include "Transform.h"
#include <iDynTree/Core/Transform.h>

namespace internal {
namespace kinematics {

    Transform::Transform(const std::string& frameName, TransformType type): m_type(type), m_frameName(frameName) {}

    Transform Transform::positionConstraint(const std::string& frameName, const iDynTree::Position &position)
    {
        Transform transform(frameName, TransformTypePosition);
        transform.m_transform.setPosition(position);
        return transform;
    }

    Transform Transform::rotationConstraint(const std::string& frameName, const iDynTree::Rotation &rotation)
    {
        Transform transform(frameName, TransformTypeRotation);
        transform.m_transform.setRotation(rotation);
        return transform;
    }

    Transform Transform::transformConstraint(const std::string& frameName, const iDynTree::Position &position, const iDynTree::Rotation &rotation)
    {
        iDynTree::Transform transform(rotation, position);
        return transformConstraint(frameName, transform);
    }

    Transform Transform::transformConstraint(const std::string& frameName, const iDynTree::Transform &_transform)
    {
        Transform transform(frameName, TransformTypeTransform);
        transform.m_transform = _transform;
        return transform;
    }

    unsigned Transform::getSize() const
    {
        switch (m_type) {
            case TransformTypePosition:
                return 3;
            case TransformTypeRotation:
                return 4;
            case TransformTypeTransform:
                return 7;
        }
    }

    Transform::TransformType Transform::getType() const { return m_type; }
    bool Transform::hasPositionConstraint() const { return m_type == TransformTypePosition || m_type == TransformTypeTransform; }
    bool Transform::hasRotationConstraint() const { return m_type == TransformTypeRotation || m_type == TransformTypeTransform; }

    const iDynTree::Position& Transform::getPosition() const { return m_transform.getPosition(); }
    const iDynTree::Rotation& Transform::getRotation() const { return m_transform.getRotation(); }
    const iDynTree::Transform& Transform::getTransform() const { return m_transform; }
    const std::string& Transform::getFrameName() const { return m_frameName; }
}
}
