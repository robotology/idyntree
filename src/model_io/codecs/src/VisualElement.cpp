// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "VisualElement.h"

#include "GeometryElement.h"
#include "MaterialElement.h"
#include "OriginElement.h"

#include <iDynTree/XMLAttribute.h>
#include <iDynTree/Utils.h>

namespace iDynTree {

    iDynTree::VisualElement::VisualInfo::VisualInfo(const std::vector<std::string>& packageDirs)
        : m_packageDirs(packageDirs)
    {
    }

    VisualElement::VisualElement(
        XMLParserState& parserState, 
        const std::string& name, 
        const std::vector<std::string>& packageDirs)
    : iDynTree::XMLElement(parserState, name)
    , m_info(packageDirs)
    {
    }

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
            return std::make_shared<OriginElement>(getParserState(), m_info.m_origin);
        } else if (name == "geometry") {
            return std::make_shared<GeometryElement>(
                getParserState(), m_info.m_solidShape, m_info.m_packageDirs);
        } else if (name == "material") {
            auto ptr = std::make_shared<MaterialElement>(getParserState(), nullptr);
            m_info.m_material = ptr->materialInfo();
            return ptr;
        }
        return std::make_shared<XMLElement>(getParserState(), name);
    }

}
