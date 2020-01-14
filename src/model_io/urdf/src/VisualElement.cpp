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

#include "VisualElement.h"

#include "GeometryElement.h"
#include "MaterialElement.h"
#include "OriginElement.h"

#include <iDynTree/XMLAttribute.h>
#include <iDynTree/Core/Utils.h>

namespace iDynTree {

    VisualElement::VisualElement(const std::string& name)
    : iDynTree::XMLElement(name){}

    const VisualElement::VisualInfo& VisualElement::visualInfo() const
    {
        return m_info;
    }

    bool VisualElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes) {

        auto found = attributes.find("name");
        if (found != attributes.end()) {
            m_info.m_name = found->second->value();
            m_info.m_nameAttributeFound = true;
        } else {
            m_info.m_name = "";
            m_info.m_nameAttributeFound = false;
        }
        return true;
    }

    std::shared_ptr<iDynTree::XMLElement> VisualElement::childElementForName(const std::string& name) {
        if (name == "origin") {
            return std::make_shared<OriginElement>(m_info.m_origin);
        } else if (name == "geometry") {
            return std::make_shared<GeometryElement>(m_info.m_solidShape);
        } else if (name == "material") {
            return std::make_shared<MaterialElement>(m_info.m_material);
        }
        return std::make_shared<XMLElement>(name);
    }

}
