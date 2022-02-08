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

#include "MaterialElement.h"

#include "URDFParsingUtils.h"

#include <iDynTree/XMLAttribute.h>

#include <iDynTree/Core/Utils.h>

namespace iDynTree {

    MaterialElement::MaterialElement(std::shared_ptr<MaterialInfo> materialInfo)
    : iDynTree::XMLElement("material")
    , m_info(materialInfo)
    {
        if (!m_info) {
            m_info = std::make_shared<MaterialInfo>();
        }
    }

    const std::shared_ptr<MaterialElement::MaterialInfo> MaterialElement::materialInfo() const { return m_info; }

    std::shared_ptr<XMLElement> MaterialElement::childElementForName(const std::string& name)
    {
        std::shared_ptr<XMLElement> element = std::make_shared<XMLElement>(name);
        if (name == "color") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes){
                auto found = attributes.find("rgba");
                if (found != attributes.end()) {
                    m_info->m_rgba = std::make_shared<iDynTree::Vector4>();
                    vector4FromString(found->second->value(), *m_info->m_rgba);
                }
                return true;
            });
        } else if (name == "texture") {
            element->setAttributeCallback([this](const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes){
                auto found = attributes.find("filename");
                if (found != attributes.end()) {
                    m_info->m_textureFilename = found->second->value();
                }
                return true;
            });
            std::string message = std::string("Texture tag not supported by iDynTree. Skipping material ") + m_info->m_name;
            reportWarning("MaterialElement", "childElementForName::texture", message.c_str());
        }
        return element;
    }

    bool MaterialElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes)
    {
        auto found = attributes.find("name");
        if (found == attributes.end()) {
            reportError("MaterialElement", "setAttributes", "Impossible to parse URDF material. Missing 'name' attribute.");
            return false;
        }
        m_info->m_name = found->second->value();
        return true;
    }
}
