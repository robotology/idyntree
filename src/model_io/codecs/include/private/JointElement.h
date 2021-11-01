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

#ifndef IDYNTREE_MODELIO_URDF_JOINTELEMENT_H
#define IDYNTREE_MODELIO_URDF_JOINTELEMENT_H

#include <iDynTree/XMLElement.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace iDynTree {
    class JointElement;
    class XMLAttribute;
    
    class IJoint;
}


class iDynTree::JointElement : public iDynTree::XMLElement {
    
public:
    // ???: Not sure where this struct should be located
    struct JointInfo {
        std::shared_ptr<IJoint> joint;
        iDynTree::Axis axis;
        std::string parentLinkName;
        std::string childLinkName;
    };
    
private:
    std::unordered_map<std::string, JointElement::JointInfo>& m_joints;
    std::unordered_map<std::string, JointElement::JointInfo>& m_fixedJoints;
    
    std::string m_jointName;
    std::string m_jointType;
    iDynTree::Transform m_jointFrame;
    iDynTree::Axis m_axis;
    std::string m_parentLink;
    std::string m_childLink;

    struct Limits {
        double positionLower;
        double positionUpper;
        double effort;
        double velocity;
    };

    std::shared_ptr<Limits> m_limits;
    
public:
    JointElement(std::unordered_map<std::string, JointElement::JointInfo>& joints,
                 std::unordered_map<std::string, JointElement::JointInfo>& fixedJoints);
    
    bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes) override;
    
    std::shared_ptr<XMLElement> childElementForName(const std::string& name) override;
    
    virtual void exitElementScope() override;
    
};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_JOINTELEMENT_H */
