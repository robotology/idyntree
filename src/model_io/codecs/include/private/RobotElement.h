// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_URDF_ROBOTELEMENT_H
#define IDYNTREE_MODELIO_URDF_ROBOTELEMENT_H

#include <iDynTree/XMLElement.h>

#include "JointElement.h"
#include "MaterialElement.h"
#include "VisualElement.h"

#include <unordered_map>
#include <vector>

//TODO: should we add a nested urdf namespace?
namespace iDynTree {
    class RobotElement;

    class SensorHelper;
    
    class Model;
    class XMLParserState;
}


class iDynTree::RobotElement : public iDynTree::XMLElement {
private:
    // Variables coming from Document, containing the intermediate state of the parsing
    iDynTree::Model& m_model;
    std::vector<std::shared_ptr<SensorHelper>>& m_sensorHelpers;
    std::unordered_map<std::string, JointElement::JointInfo>& m_joints;
    std::unordered_map<std::string, JointElement::JointInfo>& m_fixedJoints;
    std::unordered_map<std::string, MaterialElement::MaterialInfo>& m_materials;
    std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &m_visuals;
    std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &m_collisions;

public:
    explicit RobotElement(
        XMLParserState& parserState, 
        iDynTree::Model& model,
        std::vector<std::shared_ptr<SensorHelper>>& sensorHelpers,
        std::unordered_map<std::string, JointElement::JointInfo>& joints,
        std::unordered_map<std::string, JointElement::JointInfo>& fixedJoints,
        std::unordered_map<std::string, MaterialElement::MaterialInfo>& materials,
        std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &visuals,
        std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &collisions);
    
    virtual ~RobotElement();
    std::shared_ptr<iDynTree::XMLElement> childElementForName(const std::string& name) override;

    void childHasBeenParsed(std::shared_ptr<iDynTree::XMLElement>) override;

};

#endif /* end of include guard: IDYNTREE_MODELIO_URDF_ROBOTELEMENT_H */
