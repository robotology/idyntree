// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "OriginElement.h"

#include "URDFParsingUtils.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Transform.h>

namespace iDynTree {
    OriginElement::OriginElement(
        XMLParserState& parserState,
        iDynTree::Transform& jointOrigin)
    : iDynTree::XMLElement(parserState, "origin")
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
