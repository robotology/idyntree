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

#include "LinkElement.h"

#include "InertialElement.h"
#include "VisualElement.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Model/Model.h>

#include <string>
#include <unordered_map>


namespace iDynTree {
    
    LinkElement::LinkElement(iDynTree::Model &model)
    : iDynTree::XMLElement("link")
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
            return std::make_shared<InertialElement>(m_link);
        } else if (name == "visual") {
            return std::make_shared<VisualElement>("visual");
        } else if (name == "collision") {
            return std::make_shared<VisualElement>("collision");
        }
        return std::make_shared<iDynTree::XMLElement>(name);
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
