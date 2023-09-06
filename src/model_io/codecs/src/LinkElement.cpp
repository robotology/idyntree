// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "LinkElement.h"

#include "InertialElement.h"
#include "VisualElement.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Model.h>

#include <string>
#include <unordered_map>


namespace iDynTree {
    
    LinkElement::LinkElement(
        XMLParserState& parserState, 
        iDynTree::Model &model)
    : iDynTree::XMLElement(parserState, "link")
    , m_model(model)
    {
        iDynTree::SpatialInertia zeroInertia = iDynTree::SpatialInertia::Zero();
        m_link.setInertia(zeroInertia);
    }

    const std::string& LinkElement::linkName() const { return m_linkName; }
    const std::vector<VisualElement::VisualInfo>& LinkElement::visuals() const { return m_visuals; }
    const std::vector<VisualElement::VisualInfo>& LinkElement::collisions() const { return m_collisions; }
    
    void LinkElement::exitElementScope() {
        m_model.addLink(m_linkName, m_link);
    }
    
    bool LinkElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) {
        auto found = attributes.find("name");
        if (found == attributes.end()) {
            reportError("LinkElement", "setAttributes", "No name given for a link in the model.");
            return false;
        }
        m_linkName = found->second->value();
        return true;
    }
    
    std::shared_ptr<iDynTree::XMLElement> LinkElement::childElementForName(const std::string& name) {
        if (name == "inertial") {
            return std::make_shared<InertialElement>(getParserState(), m_link);
        } else if (name == "visual") {
            return std::make_shared<VisualElement>(getParserState(), "visual", m_model.getPackageDirs());
        } else if (name == "collision") {
            return std::make_shared<VisualElement>(getParserState(), "collision", m_model.getPackageDirs());
        }
        return std::make_shared<iDynTree::XMLElement>(getParserState(), name);
    }

    void LinkElement::childHasBeenParsed(std::shared_ptr<iDynTree::XMLElement> child)
    {
        std::vector<VisualElement::VisualInfo>* visualContainer = nullptr;
        if (child->name() == "visual") {
            visualContainer = &m_visuals;
        } else if (child->name() == "collision") {
            visualContainer = &m_collisions;
        } else {
            return;
        }
        std::shared_ptr<VisualElement> visualElement = std::dynamic_pointer_cast<VisualElement>(child);
        if (!visualElement) {
            reportError("LinkElement", "childHasBeenParsed", "Expecting a VisualElement for visual/collision child. Got another element instead.");
            return;
        }
        visualContainer->push_back(visualElement->visualInfo());
    }
    
}
