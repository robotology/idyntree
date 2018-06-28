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

#include "RobotElement.h"

#include "LinkElement.h"
#include "JointElement.h"
#include "SensorElement.h"
#include "MaterialElement.h"

#include <iDynTree/Model/Model.h>
#include <iDynTree/Sensors/Sensors.h>

#include <unordered_set>

namespace iDynTree {

    RobotElement::RobotElement(iDynTree::Model& model,
                               std::vector<std::shared_ptr<SensorHelper>>& sensorHelpers,
                               std::unordered_map<std::string, JointElement::JointInfo>& joints,
                               std::unordered_map<std::string, JointElement::JointInfo>& fixedJoints,
                               std::unordered_map<std::string, MaterialElement::MaterialInfo>& materials,
                               std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &visuals,
                               std::unordered_map<std::string, std::vector<VisualElement::VisualInfo>> &collisions)
    : iDynTree::XMLElement("robot")
    , m_model(model)
    , m_sensorHelpers(sensorHelpers)
    , m_joints(joints)
    , m_fixedJoints(fixedJoints)
    , m_materials(materials)
    , m_visuals(visuals)
    , m_collisions(collisions) {}

    RobotElement::~RobotElement() {}

    std::shared_ptr<iDynTree::XMLElement> RobotElement::childElementForName(const std::string& name)
    {
        if (name == "link") {
            return std::make_shared<LinkElement>(m_model);
        } else if (name == "joint") {
            return std::make_shared<JointElement>(m_joints, m_fixedJoints);
        } else if (name == "sensor") {
            return std::make_shared<SensorElement>(m_sensorHelpers);
        } else if (name == "material") {
            // These materials constitute a model-level database of materials
            // TODO: the result of the parsing should be added to the database
            return std::make_shared<MaterialElement>(nullptr);
        }
        //TODO: What to do with an unexpected tag?
        // we would like to support two modes: strict (raise an error for an unexpected tag)
        // non-strict: just ignore the tag (current mode).

        // Then the questions are:
        // - How can I specify the strict/non-strict mode? This function is a callback.
        //   (We might propagate an option from the URDFDocument class)
        // - Should I handle the error here or in the XML Parser class (returning a nullptr here)?
        return std::shared_ptr<iDynTree::XMLElement>(new iDynTree::XMLElement(name));
    }

    void RobotElement::childHasBeenParsed(std::shared_ptr<iDynTree::XMLElement> child)
    {
        if (child->name() == "link") {
            std::shared_ptr<LinkElement> link = std::dynamic_pointer_cast<LinkElement>(child);
            if (!link) {
                reportError("RobotElement", "childHasBeenParsed", "Expecting a LinkElement for link child. Got another element instead.");
                return;
            }
            if (!link->visuals().empty()) {
                m_visuals.insert(std::make_pair(link->linkName(), link->visuals()));
            }
            if (!link->collisions().empty()) {
                m_collisions.insert(std::make_pair(link->linkName(), link->collisions()));
            }
        } else if (child->name() == "material") {
            std::shared_ptr<MaterialElement> material = std::dynamic_pointer_cast<MaterialElement>(child);
            if (!material) {
                reportError("RobotElement", "childHasBeenParsed", "Expecting a MaterialElement for link child. Got another element instead.");
                return;
            }
            std::shared_ptr<MaterialElement::MaterialInfo> info = material->materialInfo();
            if (info) {
                // ???: Converting to object to have a consistent interface
                m_materials.insert(std::make_pair(info->m_name, *info));
            }
        }
    }

}

