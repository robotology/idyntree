/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Author: Francesco Romano - Google LLC
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "OriginElement.h"

#include "URDFParsingUtils.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Core/Transform.h>

namespace iDynTree {
    OriginElement::OriginElement(iDynTree::Transform& jointOrigin)
    : iDynTree::XMLElement("origin")
    , m_jointOrigin(jointOrigin) {}

    bool OriginElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes)
    {
        auto xyz = attributes.find("xyz");
        if (xyz != attributes.end()) {
            Position origin;
            if (vector3FromString(xyz->second->value(), origin)) {
                m_jointOrigin.setPosition(origin);
            }
        }
        auto rpy = attributes.find("rpy");
        if (rpy != attributes.end()) {
            Rotation orientation;
            Vector3 rpyVector;
            if (vector3FromString(rpy->second->value(), rpyVector)) {
                orientation = Rotation::RPY(rpyVector(0),
                                            rpyVector(1),
                                            rpyVector(2));
                m_jointOrigin.setRotation(orientation);
            }
        }
        return true;
    }
}
